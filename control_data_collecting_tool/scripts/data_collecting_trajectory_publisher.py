#!/usr/bin/env python3

# Copyright 2024 Proxima Technology Inc, TIER IV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from autoware_adapi_v1_msgs.msg import OperationModeState
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class DataCollectingTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("data_collecting_trajectory_publisher")

        self.trajectory_for_collecting_data_pub_ = self.create_publisher(
            Trajectory,
            "/data_collecting_trajectory",
            1,
        )

        self.data_collecting_trajectory_marker_array_pub_ = self.create_publisher(
            MarkerArray,
            "/data_collecting_trajectory_marker_array",
            1,
        )

        self.sub_odometry_ = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.onOdometry,
            1,
        )
        self.sub_odometry_
        
        self.sub_operation_mode_ = self.create_subscription(
            OperationModeState,
            "/system/operation_mode/state",
            self.onOperationMode,
            1,
        )
        self.sub_operation_mode_

        self.sub_data_collecting_area_ = self.create_subscription(
            PolygonStamped,
            "/data_collecting_area",
            self.onDataCollectingArea,
            1,
        )
        self.sub_data_collecting_area_

        self.timer_period_callback = 0.03  # 30ms

        self.timer = self.create_timer(self.timer_period_callback, self.timer_callback)

        self._present_kinematic_state = None
        self._data_collecting_area_polygon = None

        # only for prototype
        self.previous_cosx = None
        self.previous_sinx = None

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onOperationMode(self, msg):
        self._present_operation_mode = msg

    def onDataCollectingArea(self, msg):
        self._data_collecting_area_polygon = msg

    def timer_callback(self):
    
        if (self._present_kinematic_state is not None and self._data_collecting_area_polygon is not None):
            present_position = np.array(
                [
                    self._present_kinematic_state.pose.pose.position.x,
                    self._present_kinematic_state.pose.pose.position.y,
                    self._present_kinematic_state.pose.pose.position.z,
                ]
            )
            present_orientation = np.array(
                [
                    self._present_kinematic_state.pose.pose.orientation.x,
                    self._present_kinematic_state.pose.pose.orientation.y,
                    self._present_kinematic_state.pose.pose.orientation.z,
                    self._present_kinematic_state.pose.pose.orientation.w,
                ]
            )
            
            data_collecting_area = np.array([np.array([self._data_collecting_area_polygon.polygon.points[i].x,
                                  self._data_collecting_area_polygon.polygon.points[i].y,
                                  self._data_collecting_area_polygon.polygon.points[i].z]) for i in range(4)])
            marker_array = MarkerArray()

            """

            marker_corner = Marker()
            marker_corner.type = 4
            marker_corner.header.frame_id = "map"
            marker_corner.action = marker_corner.ADD

            marker_corner.scale.x = 0.3
            marker_corner.scale.y = 0.0
            marker_corner.scale.z = 0.0

            marker_corner.color.a = 1.0
            marker_corner.color.r = 1.0
            marker_corner.color.g = 0.0
            marker_corner.color.b = 0.0

            marker_corner.lifetime.nanosec = 500000000
            marker_corner.frame_locked = True

            marker_corner.points = []

            h_width = 50
            v_width = 30
            th = -0.11

            tmp_point = Point()
            tmp_point.x = 61518.096465958 + 30
            tmp_point.y = 56220.9808963864 - 40
            tmp_point.z = present_position[2]
            marker_corner.points.append(tmp_point)

            tmp_point = Point()
            tmp_point.x = 61518.096465958 + 30 + h_width * np.cos(th)
            tmp_point.y = 56220.9808963864 - 40 + h_width * np.sin(th)
            tmp_point.z = present_position[2]
            marker_corner.points.append(tmp_point)

            tmp_point = Point()
            tmp_point.x = 61518.096465958 + 30 + h_width * np.cos(th) - v_width * np.sin(th)
            tmp_point.y = 56220.9808963864 - 40 + v_width * np.cos(th) + h_width * np.sin(th)
            tmp_point.z = present_position[2]
            marker_corner.points.append(tmp_point)

            tmp_point = Point()
            tmp_point.x = 61518.096465958 + 30 - v_width * np.sin(th)
            tmp_point.y = 56220.9808963864 - 40 + v_width * np.cos(th)
            tmp_point.z = present_position[2]
            marker_corner.points.append(tmp_point)

            marker_corner.points.append(marker_corner.points[0])

            marker_array.markers.append(marker_corner)
            """
            # 領域の中心点を目標に、Trajectoryを生成
            marker_traj = Marker()
            marker_traj.type = 4
            marker_traj.id = 0
            marker_traj.header.frame_id = "map"

            marker_traj.action = marker_traj.ADD

            marker_traj.scale.x = 0.3
            marker_traj.scale.y = 0.0
            marker_traj.scale.z = 0.0

            marker_traj.color.a = 1.0
            marker_traj.color.r = 0.0
            marker_traj.color.g = 0.0
            marker_traj.color.b = 1.0

            marker_traj.lifetime.nanosec = 500000000
            marker_traj.frame_locked = True

            marker_traj.points = []

            debug_target_point_center = np.zeros(3)
            for i in range(4):
                debug_target_point_center[0] += data_collecting_area[i,0] / 4.0
                debug_target_point_center[1] += data_collecting_area[i,1] / 4.0
            debug_target_point_center[2] = present_position[2]

            diff = debug_target_point_center - present_position
            dist = np.sqrt((diff**2).sum())
            if dist > 0.01:
                cos2x = diff[0] / dist
                sin2x = diff[1] / dist
                cosx = np.sqrt((1 + cos2x) / 2)
                sinx = np.sqrt((1 - cos2x) / 2)
                if np.abs(2 * sinx * cosx - sin2x) > 0.01:
                    sinx = -sinx
                self.previous_cosx = cosx
                self.previous_sinx = sinx
            else:
                cosx = self.previous_cosx
                sinx = self.previous_sinx

            traj_point_num = int(dist) + 1

            tmp_traj = Trajectory()

            for i in range(traj_point_num):
                tmp_traj_point = TrajectoryPoint()
                tmp_traj_point.pose.position.x = (
                    present_position[0] + (i / traj_point_num) * diff[0]
                )
                tmp_traj_point.pose.position.y = (
                    present_position[1] + (i / traj_point_num) * diff[1]
                )
                tmp_traj_point.pose.position.z = present_position[2]

                tmp_traj_point.pose.orientation.x = 0.0
                tmp_traj_point.pose.orientation.y = 0.0
                tmp_traj_point.pose.orientation.z = sinx
                tmp_traj_point.pose.orientation.w = cosx

                tmp_traj_point.longitudinal_velocity_mps = 2.5
                if dist < 5:
                    tmp_traj_point.longitudinal_velocity_mps = 0.0

                tmp_traj.points.append(tmp_traj_point)

                tmp_marker_point = Point()
                tmp_marker_point.x = tmp_traj_point.pose.position.x
                tmp_marker_point.y = tmp_traj_point.pose.position.y
                tmp_marker_point.z = tmp_traj_point.pose.position.z
                marker_traj.points.append(tmp_marker_point)

            marker_array.markers.append(marker_traj)

            self.trajectory_for_collecting_data_pub_.publish(tmp_traj)

            self.data_collecting_trajectory_marker_array_pub_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    data_collecting_trajectory_publisher = DataCollectingTrajectoryPublisher()

    rclpy.spin(data_collecting_trajectory_publisher)

    data_collecting_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
