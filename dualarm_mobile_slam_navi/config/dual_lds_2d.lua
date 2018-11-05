-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                           -- The ROS frame ID to use for publishing submaps, the parent frame of poses, usually “map”.
  tracking_frame = "base_link",                 -- The ROS frame ID of the frame that is tracked by the SLAM algorithm. If an IMU is used, it should be at its position, although it might be rotated. A common choice is “imu_link”.
  published_frame = "base_link",                    -- The ROS frame ID to use as the child frame for publishing poses. For example “odom” if an “odom” frame is supplied by a different part of the system. In this case the pose of “odom” in the map_frame will be published. Otherwise, setting it to “base_link” is likely appropriate.
  odom_frame = "odom",                         -- Only used if provide_odom_frame is true. The frame between published_frame and map_frame to be used for publishing the (non-loop-closed) local SLAM result. Usually “odom”.
  provide_odom_frame = true,                  -- If enabled, the local, non-loop-closed, continuous pose will be published as the odom_frame in the map_frame.
  publish_frame_projected_to_2d = false,
  use_odometry = true,                         -- If enabled, subscribes to nav_msgs/Odometry on the topic “odom”. Odometry must be provided in this case, and the information will be included in SLAM.
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,                         -- Number of laser scan topics to subscribe to. Subscribes to sensor_msgs/LaserScan on the “scan” topic for one laser scanner, or topics “scan_1”, “scan_2”, etc. for multiple laser scanners.
  num_multi_echo_laser_scans = 0,              -- Number of multi-echo laser scan topics to subscribe to. Subscribes to sensor_msgs/MultiEchoLaserScan on the “echoes” topic for one laser scanner, or topics “echoes_1”, “echoes_2”, etc. for multiple laser scanners.
  num_subdivisions_per_laser_scan = 1,         -- Number of point clouds to split each received (multi-echo) laser scan into. Subdividing a scan makes it possible to unwarp scans acquired while the scanners are moving. There is a corresponding trajectory builder option to accumulate the subdivided scans into a point cloud that will be used for scan matching.
  num_point_clouds = 0,                        -- Number of point cloud topics to subscribe to. Subscribes to sensor_msgs/PointCloud2 on the “points2” topic for one rangefinder, or topics “points2_1”, “points2_2”, etc. for multiple rangefinders.
  lookup_transform_timeout_sec = 0.2,          -- Timeout in seconds to use for looking up transforms using tf2.
  submap_publish_period_sec = 0.3,             -- Interval in seconds at which to publish the submap poses, e.g. 0.3 seconds.
  pose_publish_period_sec = 1e-2,              -- Interval in seconds at which to publish poses, e.g. 5e-3 for a frequency of 200 Hz.
  trajectory_publish_period_sec = 30e-3,       -- Interval in seconds at which to publish the trajectory markers, e.g. 30e-3 for 30 milliseconds.
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.1,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.1,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
