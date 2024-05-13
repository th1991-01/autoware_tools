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
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry
import numpy as np
from numpy import arctan
from numpy import cos
from numpy import pi
from numpy import sin
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def get_trajectory_points(
    long_side_length: float, short_side_length: float, step: float, total_distance: float
):
    a = short_side_length
    b = long_side_length

    t_array = np.arange(start=0.0, stop=total_distance, step=step).astype("float")
    x = t_array.copy()
    y = t_array.copy()
    yaw = t_array.copy()

    # Boundary points between circular and linear trajectory
    _A = [-(b - a) / 2, a / 2]
    _B = [(b - a) / 2, a / 2]
    C = [-(b - a) / 2, -a / 2]
    D = [(b - a) / 2, -a / 2]

    _O = [0.0, 0.0]  # origin
    R = a / 2  # radious of the circle
    OL = [-(b - a) / 2, 0]  # center of the left circle
    OR = [(b - a) / 2, 0]  # center of the right circle
    OB = np.sqrt((b - a) ** 2 + a**2) / 2  # half length of the linear trajectory
    AD = 2 * OB
    θB = arctan(a / (b - a))  # Angle that OB makes with respect to x-axis
    BD = pi * a / 2  # the length of arc BD
    AC = BD
    CO = OB

    i_end = t_array.shape[0]
    for i, t in enumerate(t_array):
        if t > OB + BD + AD + AC + CO:
            i_end = i
            break
        if 0 <= t and t <= OB:
            x[i] = (b - a) * t / (2 * OB)
            y[i] = a * t / (2 * OB)
            yaw[i] = θB
        if OB <= t and t <= OB + BD:
            t1 = t - OB
            t1_rad = t1 / R
            x[i] = OR[0] + R * cos(pi / 2 - t1_rad)
            y[i] = OR[1] + R * sin(pi / 2 - t1_rad)
            yaw[i] = -t1_rad
        if OB + BD <= t and t <= OB + BD + AD:
            t2 = t - (OB + BD)
            x[i] = D[0] - (b - a) * t2 / (2 * OB)
            y[i] = D[1] + a * t2 / (2 * OB)
            yaw[i] = pi - θB
        if OB + BD + AD <= t and t <= OB + BD + AD + AC:
            t3 = t - (OB + BD + AD)
            t3_rad = t3 / R
            x[i] = OL[0] + R * cos(pi / 2 + t3_rad)
            y[i] = OL[1] + R * sin(pi / 2 + t3_rad)
            yaw[i] = pi + t3_rad
        if OB + BD + AD + AC <= t and t <= OB + BD + AD + AC + CO:
            t4 = t - (OB + BD + AD + AC)
            x[i] = C[0] + (b - a) * t4 / (2 * OB)
            y[i] = C[1] + a * t4 / (2 * OB)
            yaw[i] = θB

    # drop rest
    x = x[:i_end]
    y = y[:i_end]
    yaw = yaw[:i_end]
    return np.array([x, y]).T, yaw


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

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onDataCollectingArea(self, msg):
        self._data_collecting_area_polygon = msg

    def timer_callback(self):
        if (
            self._present_kinematic_state is not None
            and self._data_collecting_area_polygon is not None
        ):
            # TODP: データ収集領域をうまく選択できなかった場合のエラーチェックを入れる

            # [0] receive data from topic
            present_position = np.array(
                [
                    self._present_kinematic_state.pose.pose.position.x,
                    self._present_kinematic_state.pose.pose.position.y,
                    self._present_kinematic_state.pose.pose.position.z,
                ]
            )

            data_collecting_area = np.array(
                [
                    np.array(
                        [
                            self._data_collecting_area_polygon.polygon.points[i].x,
                            self._data_collecting_area_polygon.polygon.points[i].y,
                            self._data_collecting_area_polygon.polygon.points[i].z,
                        ]
                    )
                    for i in range(4)
                ]
            )

            # [1] compute an approximate rectangle
            l1 = np.sqrt(((data_collecting_area[0, :2] - data_collecting_area[1, :2]) ** 2).sum())
            l2 = np.sqrt(((data_collecting_area[1, :2] - data_collecting_area[2, :2]) ** 2).sum())
            l3 = np.sqrt(((data_collecting_area[2, :2] - data_collecting_area[3, :2]) ** 2).sum())
            l4 = np.sqrt(((data_collecting_area[3, :2] - data_collecting_area[0, :2]) ** 2).sum())
            la = (l1 + l3) / 2
            lb = (l2 + l4) / 2
            ld = np.sqrt(la**2 + lb**2)

            rectangle_center_position = np.zeros(2)
            for i in range(4):
                rectangle_center_position[0] += data_collecting_area[i, 0] / 4.0
                rectangle_center_position[1] += data_collecting_area[i, 1] / 4.0

            vec_from_center_to_point0_data = data_collecting_area[0, :2] - rectangle_center_position
            vec_from_center_to_point1_data = data_collecting_area[1, :2] - rectangle_center_position
            unitvec_from_center_to_point0_data = vec_from_center_to_point0_data / np.sqrt(
                (vec_from_center_to_point0_data**2).sum()
            )
            unitvec_from_center_to_point1_data = vec_from_center_to_point1_data / np.sqrt(
                (vec_from_center_to_point1_data**2).sum()
            )

            # [2] compute trajectory point
            if la > lb:
                long_side_length = la
                short_side_length = lb
                vec_long_side = (
                    -unitvec_from_center_to_point0_data + unitvec_from_center_to_point1_data
                )
            else:
                long_side_length = lb
                short_side_length = la
                vec_long_side = (
                    unitvec_from_center_to_point0_data + unitvec_from_center_to_point1_data
                )
            unitvec_long_side = vec_long_side / np.sqrt((vec_long_side**2).sum())
            if unitvec_long_side[1] < 0:
                unitvec_long_side *= -1
            yaw_offset = np.arccos(unitvec_long_side[0])
            if yaw_offset > pi / 2:
                yaw_offset -= pi

            long_side_margin = 5
            long_side_margin = 5
            step = 0.1
            total_distance = ld * 3.5
            trajectory_position_data, trajectory_yaw_data = get_trajectory_points(
                max(long_side_length - long_side_margin, 1.0),
                max(short_side_length - long_side_margin, 1.0),
                step,
                total_distance,
            )

            rot_matrix = np.array(
                [
                    [np.cos(yaw_offset), -np.sin(yaw_offset)],
                    [np.sin(yaw_offset), np.cos(yaw_offset)],
                ]
            )
            trajectory_position_data = (rot_matrix @ trajectory_position_data.T).T
            trajectory_position_data += rectangle_center_position

            marker_array = MarkerArray()
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

            tmp_traj = Trajectory()

            for i in range(len(trajectory_position_data)):
                tmp_traj_point = TrajectoryPoint()
                tmp_traj_point.pose.position.x = trajectory_position_data[i, 0]
                tmp_traj_point.pose.position.y = trajectory_position_data[i, 1]
                tmp_traj_point.pose.position.z = present_position[2]

                tmp_traj_point.pose.orientation.x = 0.0
                tmp_traj_point.pose.orientation.y = 0.0
                tmp_traj_point.pose.orientation.z = np.sin(trajectory_yaw_data[i] / 2)
                tmp_traj_point.pose.orientation.w = np.cos(trajectory_yaw_data[i] / 2)

                tmp_traj_point.longitudinal_velocity_mps = 2.5
                # if dist < 5:
                #    tmp_traj_point.longitudinal_velocity_mps = 0.0

                tmp_traj.points.append(tmp_traj_point)

            marker_downsampling = 3
            for i in range(len(trajectory_position_data) // marker_downsampling):
                tmp_marker_point = Point()
                tmp_marker_point.x = trajectory_position_data[i * marker_downsampling, 0]
                tmp_marker_point.y = trajectory_position_data[i * marker_downsampling, 1]
                tmp_marker_point.z = 0.0
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
