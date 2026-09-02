// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "livox_laser_simulation/lidar_reconfig.hpp"
#include "livox_laser_simulation/msg/custom_msg.hpp"
#include "livox_laser_simulation/msg/custom_point.hpp"

namespace livox_laser_simulation
{
using msg::CustomMsg;
using msg::CustomPoint;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr uint16_t kPointCloud = 0;
constexpr uint16_t kPointCloud2Xyz = 1;
constexpr uint16_t kPointCloud2Xyzrtlt = 2;
constexpr uint16_t kCustomMsg = 3;

struct ScanPoint
{
  double time;
  double azimuth;
  double zenith;
  uint8_t line;
};

bool ParseCsv(const std::string &path, std::vector<ScanPoint> &points)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    return false;
  }

  std::string row;
  while (std::getline(input, row)) {
    std::replace(row.begin(), row.end(), ',', ' ');
    std::istringstream values(row);
    double time;
    double azimuth;
    double zenith;
    if (!(values >> time >> azimuth >> zenith)) {
      continue;
    }
    // The legacy CSV stores zenith in [0, 180] degrees. Gazebo's vertical
    // LiDAR angle is pitch, whose zero is the horizon.
    points.push_back({time, azimuth * kPi / 180.0,
        zenith * kPi / 180.0 - kPi / 2.0,
        static_cast<uint8_t>(points.size() % 4)});
  }
  return !points.empty();
}

std::string ResolveCsvPath(const std::string &path)
{
  if (path.rfind("model://", 0) == 0) {
    // Gazebo Sim resources are configured through GZ_SIM_RESOURCE_PATH,
    // whereas gz-common defaults to GZ_FILE_PATH.
    gz::common::SystemPaths paths;
    paths.SetFilePathEnv("GZ_SIM_RESOURCE_PATH");
    return paths.FindFileURI(path);
  }
  return path;
}

void AddField(sensor_msgs::msg::PointCloud2 &cloud, const std::string &name,
    uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  cloud.fields.push_back(std::move(field));
}

void WriteFloat(std::vector<uint8_t> &data, size_t offset, float value)
{
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

void WriteDouble(std::vector<uint8_t> &data, size_t offset, double value)
{
  std::memcpy(data.data() + offset, &value, sizeof(value));
}
}  // namespace

/// \brief Converts a dense Gazebo Sim LiDAR scan into the Mid-360 scan order.
///
/// Gazebo Sim LiDAR uses a rectangular angular grid, while Mid-360's pattern
/// is an ordered list of arbitrary rays. The source LiDAR must therefore span
/// the Mid-360's field of view; each CSV ray picks the closest grid sample.
///
/// Every SDF knob is re-exposed as a ROS 2 parameter, so `ros2 param set` or
/// rqt_reconfigure can retune the output (point count, range filter, scan
/// pattern) without editing the SDF file or restarting the simulation.
class LivoxGzSystem final : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPostUpdate
{
public:
  ~LivoxGzSystem() override
  {
    if (this->executor_ != nullptr) {
      this->executor_->cancel();
    }
    if (this->spin_thread_.joinable()) {
      this->spin_thread_.join();
    }
  }

  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    this->entity_ = entity;

    this->frame_id_ = sdf->Get<std::string>("frame_name", "livox_link").first;
    this->ros_topic_ = sdf->Get<std::string>("ros_topic", "/livox/lidar").first;
    this->point_type_ = static_cast<uint16_t>(sdf->Get<int>(
        "publish_pointcloud_type", static_cast<int>(kPointCloud2Xyzrtlt)).first);
    this->params_.samples = std::max<uint32_t>(1,
        sdf->Get<uint32_t>("samples", 18000U).first);
    this->params_.downsample = std::max<uint32_t>(1,
        sdf->Get<uint32_t>("downsample", 1U).first);
    this->params_.min_range = sdf->Get<double>("min_range", 0.1).first;
    this->params_.max_range = sdf->Get<double>("max_range", 45.0).first;
    this->params_.publish_max_range = sdf->Get<bool>("publish_max_range", false).first;

    const auto node_name = sdf->Get<std::string>("ros_node_name",
        "livox_mid360_gz").first;
    // Stamp the clouds with the simulation clock so that RViz, the bridged
    // /tf (also sim-stamped) and the point clouds share one time base.
    // Requires the gz /clock topic to be bridged to ROS; without /clock the
    // node clock stays at 0.
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("use_sim_time",
        sdf->Get<bool>("use_sim_time", true).first);
    this->ros_node_ = std::make_shared<rclcpp::Node>(node_name, node_options);
    switch (this->point_type_) {
      case kPointCloud:
        this->point_cloud_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::PointCloud>(
          this->ros_topic_, rclcpp::SensorDataQoS());
        break;
      case kPointCloud2Xyz:
      case kPointCloud2Xyzrtlt:
        this->point_cloud2_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          this->ros_topic_, rclcpp::SensorDataQoS());
        break;
      case kCustomMsg:
        this->custom_pub_ = this->ros_node_->create_publisher<CustomMsg>(
          this->ros_topic_, rclcpp::SensorDataQoS());
        break;
      default:
        RCLCPP_ERROR(this->ros_node_->get_logger(),
          "Unsupported publish_pointcloud_type %u", this->point_type_);
        return;
    }

    this->params_.csv_file = sdf->Get<std::string>("csv_file",
        "model://Mid360/scan_mode/mid360-real-centr.csv").first;
    const std::string csv_path = ResolveCsvPath(this->params_.csv_file);
    if (!ParseCsv(csv_path, this->pattern_)) {
      RCLCPP_ERROR(this->ros_node_->get_logger(),
        "Unable to read Mid-360 scan CSV: %s", csv_path.c_str());
      return;
    }

    this->gz_scan_topic_ = sdf->Get<std::string>("gz_scan_topic", "").first;
    if (this->gz_scan_topic_.empty()) {
      RCLCPP_ERROR(this->ros_node_->get_logger(),
        "<gz_scan_topic> is required (it must match the gpu_lidar <topic>)");
      return;
    }
    if (!this->gz_node_.Subscribe(this->gz_scan_topic_, &LivoxGzSystem::OnScan, this)) {
      RCLCPP_ERROR(this->ros_node_->get_logger(),
        "Could not subscribe to Gazebo LiDAR topic: %s", this->gz_scan_topic_.c_str());
      return;
    }

    // Expose the SDF knobs as ROS 2 parameters for dynamic reconfiguration.
    DeclareLidarParameters(this->ros_node_, this->params_);
    this->param_callback_handle_ =
      this->ros_node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        return this->OnParametersChanged(parameters);
      });

    // Parameter services only answer while an executor spins the node. The
    // scan data itself still arrives on the Gazebo transport thread, so the
    // shared state is guarded by params_mutex_.
    this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->executor_->add_node(this->ros_node_);
    this->spin_thread_ = std::thread([this]() { this->executor_->spin(); });

    // Optional ground-truth TF publisher: wheel odometry is meaningless for
    // skid-steer robots (measured: 168 deg yaw error on a single turn), so
    // in simulation publish the true model pose read from the ECM in
    // PostUpdate. Enabled by <tf_child_frame> in the plugin SDF.
    this->tf_child_frame_ = sdf->Get<std::string>("tf_child_frame", "").first;
    if (!this->tf_child_frame_.empty()) {
      this->tf_pub_ = this->ros_node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
      RCLCPP_INFO(this->ros_node_->get_logger(),
        "Publishing ground-truth TF: odom -> %s", this->tf_child_frame_.c_str());
    }

    RCLCPP_INFO(this->ros_node_->get_logger(),
      "Mid-360 adapter ready: %zu CSV rays, %u output rays / scan, input=%s, output=%s",
      this->pattern_.size(), this->params_.samples, this->gz_scan_topic_.c_str(),
      this->ros_topic_.c_str());
  }

private:
  struct OutputPoint
  {
    float x;
    float y;
    float z;
    float intensity;
    uint8_t line;
    uint32_t offset_time;
  };

  rcl_interfaces::msg::SetParametersResult OnParametersChanged(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    // Fold the update into a candidate first; shared state is only touched
    // after every check passed, so a rejected update changes nothing.
    LidarDynamicParams candidate;
    {
      std::lock_guard<std::mutex> lock(this->params_mutex_);
      candidate = this->params_;
    }
    bool csv_changed = false;
    auto result = UpdateLidarParameters(parameters, candidate, &csv_changed);
    if (!result.successful) {
      return result;
    }
    std::vector<ScanPoint> pattern;
    if (csv_changed) {
      const std::string csv_path = ResolveCsvPath(candidate.csv_file);
      if (!ParseCsv(csv_path, pattern)) {
        result.successful = false;
        result.reason = "unable to read scan pattern: " + candidate.csv_file;
        return result;
      }
    }
    {
      std::lock_guard<std::mutex> lock(this->params_mutex_);
      this->params_ = candidate;
      if (csv_changed) {
        this->pattern_ = std::move(pattern);
      }
      // Restart the pattern cursor so the next scan starts from ray zero.
      this->start_index_ = 0;
    }
    RCLCPP_INFO(this->ros_node_->get_logger(),
      "Reconfigured: samples=%u, downsample=%u, min_range=%.2f, max_range=%.2f, csv_file=%s",
      candidate.samples, candidate.downsample, candidate.min_range,
      candidate.max_range, candidate.csv_file.c_str());
    return result;
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->tf_pub_) {
      return;
    }
    const auto *pose = ecm.Component<gz::sim::components::Pose>(this->entity_);
    if (pose == nullptr) {
      return;
    }
    // Throttle to ~30 Hz; stamped with the exact simulation time.
    const double sim_s = std::chrono::duration<double>(info.simTime).count();
    if (sim_s - this->tf_last_pub_s_ < 0.033 && this->tf_last_pub_s_ >= 0.0) {
      return;
    }
    this->tf_last_pub_s_ = sim_s;

    tf2_msgs::msg::TFMessage msg;
    geometry_msgs::msg::TransformStamped t;
    const int64_t nsec = info.simTime.count();
    t.header.stamp.sec = static_cast<int32_t>(nsec / 1000000000LL);
    t.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000LL);
    t.header.frame_id = "odom";
    t.child_frame_id = this->tf_child_frame_;
    const auto &p = pose->Data();
    t.transform.translation.x = p.Pos().X();
    t.transform.translation.y = p.Pos().Y();
    t.transform.translation.z = p.Pos().Z();
    t.transform.rotation.x = p.Rot().X();
    t.transform.rotation.y = p.Rot().Y();
    t.transform.rotation.z = p.Rot().Z();
    t.transform.rotation.w = p.Rot().W();
    msg.transforms.push_back(std::move(t));
    this->tf_pub_->publish(msg);
  }

  void OnScan(const gz::msgs::LaserScan &scan)
  {
    if (!this->ros_node_) {
      return;
    }
    std::lock_guard<std::mutex> lock(this->params_mutex_);
    if (this->pattern_.empty()) {
      return;
    }
    const uint32_t horizontal_count = scan.count();
    const uint32_t vertical_count = scan.vertical_count();
    if (horizontal_count == 0 || vertical_count == 0 || scan.ranges_size() == 0) {
      return;
    }

    // Stamp the cloud with the frame's capture time carried by the gz
    // message, not with "now": the processing and /clock relay lag would
    // otherwise associate each frame with a pose from the wrong instant and
    // smear the accumulated cloud while the vehicle moves (a rotating car
    // turns its own walls into circles).
    const gz::msgs::Time &scan_time = scan.header().stamp();
    const rclcpp::Time stamp = (scan_time.sec() == 0 && scan_time.nsec() == 0)
      ? this->ros_node_->now()
      : rclcpp::Time(scan_time.sec(), scan_time.nsec(), RCL_ROS_TIME);
    const size_t pattern_size = this->pattern_.size();
    const size_t requested = std::min<size_t>(this->params_.samples, pattern_size);
    std::vector<OutputPoint> output;
    output.reserve(requested / this->params_.downsample + 1);

    const double horizontal_step = scan.angle_step();
    const double vertical_step = scan.vertical_angle_step();
    if (horizontal_step == 0.0 || vertical_step == 0.0) {
      RCLCPP_WARN(this->ros_node_->get_logger(),
        "Received a LiDAR scan without horizontal or vertical angular resolution");
      return;
    }

    for (size_t i = 0; i < requested; i += this->params_.downsample) {
      const auto &ray = this->pattern_[(this->start_index_ + i) % pattern_size];
      const long horizontal = std::lround((ray.azimuth - scan.angle_min()) / horizontal_step);
      const long vertical = std::lround((ray.zenith - scan.vertical_angle_min()) / vertical_step);
      if (horizontal < 0 || vertical < 0 || horizontal >= horizontal_count || vertical >= vertical_count) {
        continue;
      }
      const size_t scan_index = static_cast<size_t>(vertical) * horizontal_count + horizontal;
      if (scan_index >= static_cast<size_t>(scan.ranges_size())) {
        continue;
      }
      double range = scan.ranges(static_cast<int>(scan_index));
      if (!std::isfinite(range)){
        if (!this->params_.publish_max_range){
          continue;
        }
        range = scan.range_max();
      }
      if (range < this->params_.min_range || range > this->params_.max_range) {
        continue;
      }
      /*
      if (!std::isfinite(range) || range < this->params_.min_range ||
          range > this->params_.max_range ||
          (!this->params_.publish_max_range && range >= scan.range_max())) {
        continue;
      }
        */

      const double cos_pitch = std::cos(ray.zenith);
      const float intensity = scan_index < static_cast<size_t>(scan.intensities_size()) ?
        static_cast<float>(scan.intensities(static_cast<int>(scan_index))) : 0.0F;
      output.push_back({
        static_cast<float>(range * cos_pitch * std::cos(ray.azimuth)),
        static_cast<float>(range * cos_pitch * std::sin(ray.azimuth)),
        static_cast<float>(range * std::sin(ray.zenith)),
        intensity,
        ray.line,
        static_cast<uint32_t>(i * 5000U)});
    }
    this->start_index_ = (this->start_index_ + requested) % pattern_size;
    this->Publish(output, stamp);
  }

  void Publish(const std::vector<OutputPoint> &points, const rclcpp::Time &stamp)
  {
    if (this->point_type_ == kPointCloud) {
      sensor_msgs::msg::PointCloud cloud;
      cloud.header.stamp = stamp;
      cloud.header.frame_id = this->frame_id_;
      cloud.points.reserve(points.size());
      for (const auto &point : points) {
        geometry_msgs::msg::Point32 xyz;
        xyz.x = point.x;
        xyz.y = point.y;
        xyz.z = point.z;
        cloud.points.push_back(xyz);
      }
      this->point_cloud_pub_->publish(cloud);
      return;
    }
    if (this->point_type_ == kCustomMsg) {
      CustomMsg message;
      message.header.stamp = stamp;
      message.header.frame_id = this->frame_id_;
      message.timebase = static_cast<uint64_t>(stamp.nanoseconds());
      message.points.reserve(points.size());
      for (const auto &point : points) {
        CustomPoint custom;
        custom.offset_time = point.offset_time;
        custom.x = point.x;
        custom.y = point.y;
        custom.z = point.z;
        custom.reflectivity = static_cast<uint8_t>(std::clamp(point.intensity, 0.0F, 255.0F));
        custom.tag = 0x10;
        custom.line = point.line;
        message.points.push_back(custom);
      }
      message.point_num = static_cast<uint32_t>(message.points.size());
      this->custom_pub_->publish(message);
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = this->frame_id_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    AddField(cloud, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
    AddField(cloud, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
    AddField(cloud, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
    if (this->point_type_ == kPointCloud2Xyz) {
      cloud.point_step = 12;
    } else {
      AddField(cloud, "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
      AddField(cloud, "tag", 16, sensor_msgs::msg::PointField::UINT8);
      AddField(cloud, "line", 17, sensor_msgs::msg::PointField::UINT8);
      // Keep the 8-byte timestamp aligned, matching the legacy PCL point type.
      AddField(cloud, "timestamp", 24, sensor_msgs::msg::PointField::FLOAT64);
      cloud.point_step = 32;
    }
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);
    for (size_t i = 0; i < points.size(); ++i) {
      const size_t offset = i * cloud.point_step;
      const auto &point = points[i];
      WriteFloat(cloud.data, offset, point.x);
      WriteFloat(cloud.data, offset + 4, point.y);
      WriteFloat(cloud.data, offset + 8, point.z);
      if (this->point_type_ == kPointCloud2Xyzrtlt) {
        WriteFloat(cloud.data, offset + 12, point.intensity);
        cloud.data[offset + 16] = 0;
        cloud.data[offset + 17] = point.line;
        WriteDouble(cloud.data, offset + 24,
          static_cast<double>(stamp.nanoseconds() + point.offset_time));
      }
    }
    this->point_cloud2_pub_->publish(cloud);
  }

  std::mutex params_mutex_;
  gz::transport::Node gz_node_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  // Ground-truth TF (optional, enabled by <tf_child_frame>).
  gz::sim::Entity entity_{gz::sim::kNullEntity};
  double tf_last_pub_s_{-1.0};
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  std::string tf_child_frame_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_pub_;
  rclcpp::Publisher<CustomMsg>::SharedPtr custom_pub_;
  std::vector<ScanPoint> pattern_;
  std::string frame_id_;
  std::string ros_topic_;
  std::string gz_scan_topic_;
  size_t start_index_{0};
  LidarDynamicParams params_;
  uint16_t point_type_{kPointCloud2Xyzrtlt};
};
}  // namespace livox_laser_simulation

GZ_ADD_PLUGIN(livox_laser_simulation::LivoxGzSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(livox_laser_simulation::LivoxGzSystem,
  "livox_laser_simulation::LivoxGzSystem")
