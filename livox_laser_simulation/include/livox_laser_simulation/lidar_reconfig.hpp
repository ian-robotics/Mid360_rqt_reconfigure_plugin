// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#ifndef LIVOX_LASER_SIMULATION__LIDAR_RECONFIG_HPP_
#define LIVOX_LASER_SIMULATION__LIDAR_RECONFIG_HPP_

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace livox_laser_simulation
{

/// \brief Brand-agnostic dynamic parameters shared by LiDAR adapter plugins.
///
/// Every field mirrors a knob that most LiDAR SDF blocks expose, so a GUI
/// (rqt_reconfigure) or `ros2 param set` can retune any adapter reusing this
/// header without editing SDF files or restarting the simulation. The
/// adapter keeps ownership of anything brand specific (e.g. how the scan
/// pattern file is parsed).
struct LidarDynamicParams
{
  /// Output rays per scan (a subset of the scan pattern).
  uint32_t samples{18000};
  /// Emit every N-th pattern ray.
  uint32_t downsample{1};
  /// Output-side range filter [m].
  double min_range{0.1};
  double max_range{5.0};
  /// Scan-pattern file (URI or path; format is adapter specific).
  std::string csv_file;
  /// Keep rays that hit the sensor's maximum range.
  bool publish_max_range{false};
};

/// \brief Declares the parameters on a node using the current values as
/// defaults, so the parameter service reflects the SDF configuration.
inline void DeclareLidarParameters(const rclcpp::Node::SharedPtr & node,
    const LidarDynamicParams & params)
{
  rcl_interfaces::msg::ParameterDescriptor d;

  d = rcl_interfaces::msg::ParameterDescriptor();
  d.description = "每帧输出点数：从扫描模式中抽取的射线条数";
  d.integer_range.resize(1);
  d.integer_range[0].from_value = 1;
  d.integer_range[0].to_value = std::max(100000, static_cast<int>(params.samples));
  d.integer_range[0].step = 0;
  node->declare_parameter<int>("samples", static_cast<int>(params.samples), d);

  d = rcl_interfaces::msg::ParameterDescriptor();
  d.description = "抽稀系数：每 N 条模式射线输出 1 条";
  d.integer_range.resize(1);
  d.integer_range[0].from_value = 1;
  d.integer_range[0].to_value = 100;
  d.integer_range[0].step = 1;
  node->declare_parameter<int>("downsample", static_cast<int>(params.downsample), d);

  d = rcl_interfaces::msg::ParameterDescriptor();
  d.description = "最小量程过滤 [m]：丢弃比这更近的点，须小于 max_range";
  d.floating_point_range.resize(1);
  d.floating_point_range[0].from_value = 0.0;
  d.floating_point_range[0].to_value = std::max(5.0, params.min_range);
  d.floating_point_range[0].step = 0.1;
  node->declare_parameter<double>("min_range", params.min_range, d);

  d = rcl_interfaces::msg::ParameterDescriptor();
  d.description = "最大量程过滤 [m]";
  d.floating_point_range.resize(1);
  d.floating_point_range[0].from_value = 0.5;
  d.floating_point_range[0].to_value = std::max(35.0, params.max_range);
  d.floating_point_range[0].step = 0.0;
  node->declare_parameter<double>("max_range", params.max_range, d);

  d = rcl_interfaces::msg::ParameterDescriptor();
  d.description = "CSV source file path";
  node->declare_parameter<std::string>("csv_file", params.csv_file);

  d = rcl_interfaces::msg::ParameterDescriptor();
  d.description = "是否按照最大测量距离发布";
  node->declare_parameter<bool>("publish_max_range", params.publish_max_range);
}

/// \brief Atomically folds a parameter update into `params`.
///
/// Validation runs on the folded candidate; on rejection `params` is left
/// untouched and a failed SetParametersResult with a reason is returned, so
/// `ros2 param set` reports the failure. `csv_changed` (optional) tells the
/// caller whether the scan pattern must be reloaded; loading stays in the
/// adapter because the pattern format is brand specific.
inline rcl_interfaces::msg::SetParametersResult UpdateLidarParameters(
    const std::vector<rclcpp::Parameter> & parameters,
    LidarDynamicParams & params,
    bool * csv_changed = nullptr)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  LidarDynamicParams candidate = params;
  bool changed_csv = false;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "samples") {
      const auto value = parameter.as_int();
      candidate.samples = value > 0 ? static_cast<uint32_t>(value) : 0U;
    } else if (name == "downsample") {
      const auto value = parameter.as_int();
      candidate.downsample = value > 0 ? static_cast<uint32_t>(value) : 0U;
    } else if (name == "min_range") {
      candidate.min_range = parameter.as_double();
    } else if (name == "max_range") {
      candidate.max_range = parameter.as_double();
    } else if (name == "csv_file") {
      candidate.csv_file = parameter.as_string();
      changed_csv = true;
    } else if (name == "publish_max_range") {
      candidate.publish_max_range = parameter.as_bool();
    }
  }

  if (candidate.samples < 1U || candidate.downsample < 1U) {
    result.successful = false;
    result.reason = "samples and downsample must be >= 1";
    return result;
  }
  if (candidate.min_range >= candidate.max_range) {
    result.successful = false;
    result.reason = "min_range must be smaller than max_range";
    return result;
  }

  params = candidate;
  if (csv_changed != nullptr) {
    *csv_changed = changed_csv;
  }
  return result;
}

}  // namespace livox_laser_simulation

#endif  // LIVOX_LASER_SIMULATION__LIDAR_RECONFIG_HPP_
