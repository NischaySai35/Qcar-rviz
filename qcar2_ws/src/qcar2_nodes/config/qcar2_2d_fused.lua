-- QCar2 indoor mapping with LiDAR as the primary sensor and auxiliary
-- LiDAR odometry plus IMU as short-term motion constraints.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- This is also the IMU frame. Cartographer uses TF to transform each scan
  -- from base_scan, which is fixed 10 cm ahead of the vehicle origin.
  tracking_frame = "base_link",
  -- published_frame MUST be "odom", not "base_link".
  --
  -- With provide_odom_frame = false, Cartographer publishes
  -- map -> <published_frame>. Setting that to "base_link" made it publish
  -- map -> base_link directly and NO odom frame ever existed, which
  -- contradicted the line below and silently broke anything needing the
  -- standard map -> odom -> base_link tree. Nothing noticed while mapping was
  -- drive-it-yourself, because only Nav2 requires that tree -- autonomous
  -- exploration then failed with 'Invalid frame ID "odom" ... does not exist'
  -- and the car never moved.
  --
  -- With "odom", Cartographer takes its map -> base_link estimate, looks up
  -- the odom -> base_link that wheel_imu_odometry publishes, and emits the
  -- map -> odom correction -- the normal SLAM/odometry split.
  published_frame = "odom",
  odom_frame = "odom",
  -- wheel_imu_odometry.py owns odom -> base_link (it must run with
  -- publish_tf:=true; mapping.launch.py sets that). Cartographer publishes
  -- map -> odom only. These two settings are a pair: changing one without
  -- the other leaves the TF tree either broken or double-parented.
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
-- The QCar2 LiDAR reaches well beyond a small room.  Preserve free-space
-- rays through an open centre instead of leaving it permanently unknown;
-- scan matching remains protected by the conservative motion prior below.
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- In a bare room centre, parallel/distant walls do not constrain every pose
-- direction. Require stronger scan evidence before moving away from the
-- encoder/gyro prior, so an ambiguous scan cannot rotate or duplicate a map.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.12
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(12.)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 80.

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
