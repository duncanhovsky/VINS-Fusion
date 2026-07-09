# KAIST Urban 28-32 VINS-Fusion Presets

These presets target the original VINS-Fusion `vins_node` in mono+IMU mode,
using the KAIST stereo left camera only. The local project notes for urban28
recorded:

- `/stereo/left/image_raw`: `sensor_msgs/Image`, `1280x560`, `bayer_bggr8`, 10 Hz.
- `/imu/data_raw`: `sensor_msgs/Imu`, 100 Hz.
- `/gps/fix`: `sensor_msgs/NavSatFix`, 5 Hz.

The launch file converts raw KAIST IMU samples from FLU to FRD on
`/imu/data_frd`, then runs VINS-Fusion against `/imu/data_frd` and
`/stereo/left/image_mono`. The mono image topic is produced from
`/stereo/left/image_raw` with `image_proc/debayer`, because the original
VINS-Fusion image path requests `mono8` from `cv_bridge`.

The configs do not use `/gps/fix` directly. Optional `global_fusion_node`
can be enabled from the launch wrapper and remaps KAIST `/gps/fix` to the
original VINS-Fusion `/gps` subscription.

Before quantitative evaluation, verify the exact bag with `rosbag info` and
replace camera/IMU calibration if your urban29-32 bags use per-sequence
calibration different from the urban28-style local reference.

