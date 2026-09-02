# Livox Mid-360 for ROS 2 + Gazebo Sim

ROS 2 Humble / Gazebo Harmonic port of the Livox Mid-360 Gazebo Classic plugin.
The Gazebo Sim system plugin subscribes to `/livox/mid360/scan` and publishes
the ROS 2 point cloud on `/livox/lidar` directly; `ros_gz_bridge` is not needed.


## Build
Download:
```bash
mkdir ~/ros2_ws/src -p
cd ~/ros2_ws/src
git clone https://github.com/Tfly6/Mid360_px4_sim_plugin.git
cd Mid360_px4_sim_plugin/
git checkout ros2
```
Build from the workspace root:

```bash
cd ~/ros2_ws/
colcon build --packages-select livox_laser_simulation
source install/setup.bash
```

## Key files

Paths relative to the repository root (`Mid360_px4_sim_plugin/`):

| File | Function |
| --- | --- |
| `livox_laser_simulation/src/livox_gz_system.cpp` | Core gz-sim system plugin (the adapter): subscribes to the gpu_lidar topic, reorders the dense grid scan into the Mid-360 CSV pattern, publishes the ROS 2 cloud; owns the parameter callback, the output-side range filter, the parameter-service spin thread and the shared-state mutex |
| `livox_laser_simulation/include/livox_laser_simulation/lidar_reconfig.hpp` | Generic, brand-agnostic parameter layer (`LidarDynamicParams`, `DeclareLidarParameters`, `UpdateLidarParameters`); reusable by adapters for other LiDARs |
| `livox_laser_simulation/CMakeLists.txt` | Build script: message generation (`rosidl_generate_interfaces` + `rosidl_get_typesupport_target`) and the `rcl_interfaces` dependency for the parameter services |
| `livox_laser_simulation/package.xml` | Package manifest (dependencies, incl. `rcl_interfaces`) |
| `livox_laser_simulation/models/Mid360/Mid360.sdf` | LiDAR model: `gpu_lidar` sensor block (physical FOV / range) + system plugin block (adapter knobs and interface wiring) |
| `livox_laser_simulation/worlds/mid360_gz.sdf` | Simulation world: required system plugins plus test obstacles (wall, pillars, crate) for point-cloud verification |
| `livox_laser_simulation/launch/mid360_gz.launch.py` | Launch file: sets `GZ_SIM_RESOURCE_PATH` / `GZ_SIM_SYSTEM_PLUGIN_PATH`, runs `gz sim` |

## Mid-360 world

```bash
cd ~/ros2_ws/
source install/setup.bash
ros2 launch livox_laser_simulation mid360_gz.launch.py
rviz2 -d "$(ros2 pkg prefix livox_laser_simulation)/share/livox_laser_simulation/rviz/test.rviz"
```

`publish_pointcloud_type` keeps the legacy numeric contract:

| Value | ROS 2 output |
| --- | --- |
| 0 | `sensor_msgs/msg/PointCloud` |
| 1 | `sensor_msgs/msg/PointCloud2` (`x`, `y`, `z`) |
| 2 | `sensor_msgs/msg/PointCloud2` (`x`, `y`, `z`, `intensity`, `tag`, `line`, `timestamp`) |
| 3 | `livox_laser_simulation/msg/CustomMsg` |

## Parameters, classified by how you change them

### Method 1 — Runtime, command line (no restart, no SDF edit)

The plugin re-exposes its SDF knobs as ROS 2 parameters on the adapter node
(`/livox_mid360_gz`, override via `<ros_node_name>`):

```bash
ros2 param set /livox_mid360_gz max_range 25.0   # cloud shrinks in RViz instantly
ros2 param set /livox_mid360_gz samples 5000
ros2 param get /livox_mid360_gz max_range
```

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `samples` | int | 18000 | Output rays per scan (subset of the CSV pattern) |
| `downsample` | int | 1 | Emit every N-th pattern ray |
| `min_range` | double | 0.1 | Output-side minimum range filter [m] |
| `max_range` | double | 45.0 | Output-side maximum range filter [m] |
| `csv_file` | string | `model://Mid360/scan_mode/mid360-real-centr.csv` | Scan pattern (`model://` URI or absolute path); `avia`, `horizon`, `mid40`, `mid70`, `tele` CSVs also ship in `models/Mid360/scan_mode/` |
| `publish_max_range` | bool | false | Keep rays that hit the sensor's maximum range |

Invalid updates (`min_range >= max_range`, unreadable CSV, non-positive
`samples`/`downsample`) are rejected with a reason and leave the running
configuration untouched.

### Method 2 — Runtime, GUI (same parameters as Method 1)

```bash
ros2 run rqt_reconfigure rqt_reconfigure   # pick the /livox_mid360_gz node
```

### Method 3 — SDF plugin block (initial values; edit + relaunch)

In `models/Mid360/Mid360.sdf`, inside
`<plugin name="livox_laser_simulation::LivoxGzSystem">`. The six tags
`<samples>`, `<downsample>`, `<min_range>`, `<max_range>`, `<csv_file>`,
`<publish_max_range>` are the **initial values** of the Method 1 parameters.
Static-only tags (interface wiring, restart required):

| Tag | Current value | Meaning |
| --- | --- | --- |
| `<gz_scan_topic>` | `/livox/mid360/scan` | Input topic (must match the gpu_lidar `<topic>`) |
| `<ros_topic>` | `/livox/lidar` | Output topic |
| `<frame_name>` | `livox_link` | Point cloud `frame_id` |
| `<publish_pointcloud_type>` | 2 | 0: PointCloud / 1: PC2 XYZ / 2: PC2 XYZRTL(T) / 3: CustomMsg |
| `<ros_node_name>` | unset (defaults to `livox_mid360_gz`) | Parameter node name |

### Method 4 — SDF sensor block (physical simulation; edit + relaunch)

In `models/Mid360/Mid360.sdf`, inside `<sensor type="gpu_lidar">`. These
govern what the sensor physically casts and are **not runtime-tunable** in
the current version:

| Tag | Current value | Meaning |
| --- | --- | --- |
| `<update_rate>` | 10 Hz | Physical scan rate |
| horizontal / vertical `<samples>` | 360 / 100 | Angular grid resolution |
| horizontal `<min/max_angle>` | ±π | Horizontal FOV |
| vertical `<min/max_angle>` | ±π/2 | Vertical FOV |
| `<range><min>/<max>/<resolution>` | 0.1 / 45 / 0.02 | Physical range — the ceiling for the runtime `max_range` |
| `<visualize>` | false | Ray visualization (keep off; drawing thousands of rays slows the sim) |

How the layers relate: Methods 1/2 tune the output layer instantly; Method 3
sets the output layer's initial values and interface wiring; Method 4 defines
the physical simulation layer and caps what Method 1 can do (`max_range` can
shrink below 45 m but not exceed it). Making the sensor block runtime-tunable
(FOV / grid / update rate) requires ECM sensor changes or a world reload and
is left as future work.

Notes:

- The range filters are applied on the adapter side, so `max_range` can only
  shrink the output below the physical `<range><max>` the gpu_lidar sensor
  casts; widening beyond it requires editing the sensor SDF.
- The reusable, brand-agnostic part (`LidarDynamicParams`,
  `DeclareLidarParameters`, `UpdateLidarParameters`) lives in
  `include/livox_laser_simulation/lidar_reconfig.hpp` and can be embedded by
  adapters for other LiDARs; only the scan-pattern parsing is Mid-360
  specific.

## PX4 SITL target overlay

PX4 generates `make px4_sitl gz_<model>` targets from airframes. The supplied
script force-replaces only the two model directories (`Mid360` and
`x500_mid360`), the Livox plugin, and the `4022_gz_x500_mid360` airframe. It
adds the CMake airframe entry only when absent:

```bash
./src/Mid360_px4_sim_plugin/livox_laser_simulation/px4/install_px4_x500_mid360.sh /path/to/PX4-Autopilot
```

launching through PX4:

```bash
cd /path/to/PX4-Autopilot
make px4_sitl gz_x500_mid360
```
