# Third-party material

Behavior trees in behavior_trees/ are adapted from Navigation2's Humble branch:

- nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
- nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml

Source project: https://github.com/ros-navigation/navigation2

Navigation2 source material is licensed under the Apache License, Version 2.0.
A copy is included as LICENSE_NAV2_APACHE_2_0.txt. See the upstream repository
for authorship notices. Modifications here remove Spin/BackUp recoveries,
reduce recovery retries to two and waiting to two seconds. Default BT plugin
configuration is also derived from nav2_bringup/params/nav2_params.yaml.

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an AS IS BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.
