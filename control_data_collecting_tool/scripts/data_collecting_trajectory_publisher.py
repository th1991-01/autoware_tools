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
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

debug_matplotlib_plot_flag = False
if debug_matplotlib_plot_flag:
    import matplotlib.pyplot as plt


def getYaw(orientation_xyzw):
    return R.from_quat(orientation_xyzw.reshape(-1, 4)).as_euler("xyz")[:, 2]


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
    # _A = [-(b - a) / 2, a / 2]
    # _B = [(b - a) / 2, a / 2]
    C = [-(b - a) / 2, -a / 2]
    D = [(b - a) / 2, -a / 2]

    # _O = [0.0, 0.0]  # origin
    R = a / 2  # radious of the circle
    OL = [-(b - a) / 2, 0]  # center of the left circle
    OR = [(b - a) / 2, 0]  # center of the right circle
    OB = np.sqrt((b - a) ** 2 + a**2) / 2  # half length of the linear trajectory
    AD = 2 * OB
    θB = arctan(a / (b - a))  # Angle that OB makes with respect to x-axis
    BD = pi * a / 2  # the length of arc BD
    AC = BD
    CO = OB

    curve = t_array.copy()

    i_end = t_array.shape[0]
    for i, t in enumerate(t_array):
        if t > OB + BD + AD + AC + CO:
            i_end = i
            break
        if 0 <= t and t <= OB:
            x[i] = (b - a) * t / (2 * OB)
            y[i] = a * t / (2 * OB)
            yaw[i] = θB
            curve[i] = 1e10
        if OB <= t and t <= OB + BD:
            t1 = t - OB
            t1_rad = t1 / R
            x[i] = OR[0] + R * cos(pi / 2 - t1_rad)
            y[i] = OR[1] + R * sin(pi / 2 - t1_rad)
            yaw[i] = -t1_rad
            curve[i] = R
        if OB + BD <= t and t <= OB + BD + AD:
            t2 = t - (OB + BD)
            x[i] = D[0] - (b - a) * t2 / (2 * OB)
            y[i] = D[1] + a * t2 / (2 * OB)
            yaw[i] = pi - θB
            curve[i] = 1e10
        if OB + BD + AD <= t and t <= OB + BD + AD + AC:
            t3 = t - (OB + BD + AD)
            t3_rad = t3 / R
            x[i] = OL[0] + R * cos(pi / 2 + t3_rad)
            y[i] = OL[1] + R * sin(pi / 2 + t3_rad)
            yaw[i] = pi + t3_rad
            curve[i] = R
        if OB + BD + AD + AC <= t and t <= OB + BD + AD + AC + CO:
            t4 = t - (OB + BD + AD + AC)
            x[i] = C[0] + (b - a) * t4 / (2 * OB)
            y[i] = C[1] + a * t4 / (2 * OB)
            yaw[i] = θB
            curve[i] = 1e10
    # drop rest
    x = x[:i_end]
    y = y[:i_end]
    yaw = yaw[:i_end]
    return np.array([x, y]).T, yaw, curve[:i_end]


class DataCollectingTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("data_collecting_trajectory_publisher")

        self.declare_parameter(
            "max_lateral_accel",
            0.5,
            ParameterDescriptor(description="Max lateral acceleration limit [m/ss]"),
        )

        self.declare_parameter(
            "mov_ave_window",
            50,
            ParameterDescriptor(description="Moving average smoothing window size"),
        )

        self.declare_parameter(
            "target_longitudinal_velocity",
            3.0,
            ParameterDescriptor(description="Target longitudinal velocity [m/s]"),
        )

        self.declare_parameter(
            "longitudinal_velocity_noise_amp",
            0.1,
            ParameterDescriptor(
                description="Target longitudinal velocity additional sine noise amplitude [m/s]"
            ),
        )

        self.declare_parameter(
            "longitudinal_velocity_noise_min_period",
            5.0,
            ParameterDescriptor(
                description="Target longitudinal velocity additional sine noise minimum period [s]"
            ),
        )

        self.declare_parameter(
            "longitudinal_velocity_noise_max_period",
            20.0,
            ParameterDescriptor(
                description="Target longitudinal velocity additional sine noise maximum period [s]"
            ),
        )

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

        self.vel_noise_list = []

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onDataCollectingArea(self, msg):
        self._data_collecting_area_polygon = msg

    def timer_callback(self):
        if (
            self._present_kinematic_state is not None
            and self._data_collecting_area_polygon is not None
        ):
            # [0] receive data from topic
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
            present_linear_velocity = np.array(
                [
                    self._present_kinematic_state.twist.twist.linear.x,
                    self._present_kinematic_state.twist.twist.linear.y,
                    self._present_kinematic_state.twist.twist.linear.z,
                ]
            )

            present_yaw = getYaw(present_orientation)[0]

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
            if np.abs(la - lb) < 1e-6:
                la += 0.1  # long_side_length must not be equal to short_side_length
            ld = np.sqrt(la**2 + lb**2)
            rectangle_center_position = np.zeros(2)
            for i in range(4):
                rectangle_center_position[0] += data_collecting_area[i, 0] / 4.0
                rectangle_center_position[1] += data_collecting_area[i, 1] / 4.0

            vec_from_center_to_point0_data = data_collecting_area[0, :2] - rectangle_center_position
            vec_from_center_to_point1_data = data_collecting_area[1, :2] - rectangle_center_position
            unitvec_from_center_to_point0_data = vec_from_center_to_point0_data / (
                np.sqrt((vec_from_center_to_point0_data**2).sum()) + 1e-10
            )
            unitvec_from_center_to_point1_data = vec_from_center_to_point1_data / (
                np.sqrt((vec_from_center_to_point1_data**2).sum()) + 1e-10
            )

            # [2] compute whole trajectory
            # [2-1] generate figure eight path
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
            total_distance = ld * (1 + np.pi) * 2

            trajectory_position_data, trajectory_yaw_data, curve_data = get_trajectory_points(
                max(long_side_length - long_side_margin, 1.1),
                max(short_side_length - long_side_margin, 1.0),
                step,
                total_distance,
            )

            # [2-2] smoothing figure eight path
            smoothing_flag = True
            if smoothing_flag:
                window = self.get_parameter("mov_ave_window").get_parameter_value().integer_value
                if window < len(trajectory_position_data):
                    w = np.ones(window) / window
                    augment_data = np.vstack(
                        [
                            trajectory_position_data[-window:],
                            trajectory_position_data,
                            trajectory_position_data[:window],
                        ]
                    )
                    trajectory_position_data[:, 0] = (
                        1 * np.convolve(augment_data[:, 0], w, mode="same")[window:-window]
                    )
                    trajectory_position_data[:, 1] = (
                        1 * np.convolve(augment_data[:, 1], w, mode="same")[window:-window]
                    )
                    # NOTE: Target yaw angle trajectory is not smoothed. Please implement it if using a controller other than pure pursuit.

            # [2-3] translation and rotation of origin
            rot_matrix = np.array(
                [
                    [np.cos(yaw_offset), -np.sin(yaw_offset)],
                    [np.sin(yaw_offset), np.cos(yaw_offset)],
                ]
            )
            trajectory_position_data = (rot_matrix @ trajectory_position_data.T).T
            trajectory_position_data += rectangle_center_position
            trajectory_yaw_data += yaw_offset

            # [2-4] compute velocity
            max_lateral_accel = (
                self.get_parameter("max_lateral_accel").get_parameter_value().double_value
            )
            lateral_acc_limit = np.sqrt(max_lateral_accel * curve_data)
            target_longitudinal_velocity = (
                self.get_parameter("target_longitudinal_velocity")
                .get_parameter_value()
                .double_value
            )
            trajectory_longitudinal_velocity_data = target_longitudinal_velocity * np.ones(
                len(trajectory_position_data)
            )

            # [3] find near point index for local trajectory
            distance = np.sqrt(((trajectory_position_data - present_position[:2]) ** 2).sum(axis=1))
            index_array_near = np.argsort(distance)

            max_cos_diff_yaw_value = -1
            nearestIndex = None
            for i in range(len(index_array_near)):
                if (distance[index_array_near[0]] + step * 10) < distance[index_array_near[i]]:
                    break
                tmp_cos_diff_yaw_value = np.cos(
                    trajectory_yaw_data[index_array_near[i]] - present_yaw
                )
                if tmp_cos_diff_yaw_value > max_cos_diff_yaw_value:
                    max_cos_diff_yaw_value = 1.0 * tmp_cos_diff_yaw_value
                    nearestIndex = 1 * index_array_near[i]
            if nearestIndex is None:
                nearestIndex = index_array_near[0]

            # prepare velocity noise
            while True:
                if len(self.vel_noise_list) > len(trajectory_longitudinal_velocity_data) * 2:
                    break
                else:
                    tmp_noise_vel = (
                        np.random.rand()
                        * self.get_parameter("longitudinal_velocity_noise_amp")
                        .get_parameter_value()
                        .double_value
                    )
                    noise_min_period = (
                        self.get_parameter("longitudinal_velocity_noise_min_period")
                        .get_parameter_value()
                        .double_value
                    )
                    noise_max_period = (
                        self.get_parameter("longitudinal_velocity_noise_max_period")
                        .get_parameter_value()
                        .double_value
                    )
                    tmp_noise_period = noise_min_period + np.random.rand() * (
                        noise_max_period - noise_min_period
                    )
                    dt = self.timer_period_callback
                    noise_data_num = max(
                        4, int(tmp_noise_period / dt)
                    )  # 4 is minimum noise_data_num
                    for i in range(noise_data_num):
                        self.vel_noise_list.append(
                            tmp_noise_vel * np.sin(2.0 * np.pi * i / noise_data_num)
                        )
            self.vel_noise_list.pop(0)

            # [4] publish trajectory
            # [4-1] augment data
            aug_data_length = len(trajectory_position_data) // 4
            trajectory_position_data = np.vstack(
                [trajectory_position_data, trajectory_position_data[:aug_data_length]]
            )
            trajectory_yaw_data = np.hstack(
                [trajectory_yaw_data, trajectory_yaw_data[:aug_data_length]]
            )
            trajectory_longitudinal_velocity_data = np.hstack(
                [
                    trajectory_longitudinal_velocity_data,
                    trajectory_longitudinal_velocity_data[:aug_data_length],
                ]
            )
            trajectory_longitudinal_velocity_data[nearestIndex:] += np.array(self.vel_noise_list)[
                : len(trajectory_longitudinal_velocity_data[nearestIndex:])
            ]

            pub_traj_len = min(int(50 / step), aug_data_length)

            # debug plot
            if debug_matplotlib_plot_flag:
                plt.cla()
                step_size_array = np.sqrt(
                    ((trajectory_position_data[1:] - trajectory_position_data[:-1]) ** 2).sum(
                        axis=1
                    )
                )
                distance = np.zeros(len(trajectory_position_data))
                for i in range(1, len(trajectory_position_data)):
                    distance[i] = distance[i - 1] + step_size_array[i - 1]
                distance -= distance[nearestIndex]
                time_width_array = step_size_array / trajectory_longitudinal_velocity_data[:-1]
                timestamp = np.zeros(len(trajectory_position_data))
                for i in range(1, len(trajectory_position_data)):
                    timestamp[i] = timestamp[i - 1] + time_width_array[i - 1]
                timestamp -= timestamp[nearestIndex]

                lateral_acc_limit = np.hstack(
                    [
                        lateral_acc_limit,
                        lateral_acc_limit[:aug_data_length],
                    ]
                )
                trajectory_longitudinal_velocity_data = np.minimum(
                    trajectory_longitudinal_velocity_data, lateral_acc_limit
                )
                plt.plot(0, present_linear_velocity[0], "o", label="current_obs")
                plt.plot(
                    # distance[nearestIndex : nearestIndex + pub_traj_len],
                    timestamp[nearestIndex : nearestIndex + pub_traj_len],
                    lateral_acc_limit[nearestIndex : nearestIndex + pub_traj_len],
                    label="lateral_acc_limit",
                )
                plt.plot(
                    # distance[nearestIndex : nearestIndex + pub_traj_len],
                    timestamp[nearestIndex : nearestIndex + pub_traj_len],
                    trajectory_longitudinal_velocity_data[
                        nearestIndex : nearestIndex + pub_traj_len
                    ],
                    label="target",
                )
                plt.xlim([-0.5, 10.5])
                plt.ylim([-0.5, 12.5])

                # plt.xlabel("future driving distance [m]")
                plt.xlabel("future timestamp [s]")
                plt.ylabel("longitudinal_velocity [m/s]")
                plt.legend()
                plt.pause(0.01)

            # [4-2] publish
            tmp_traj = Trajectory()
            for i in range(pub_traj_len):
                tmp_traj_point = TrajectoryPoint()
                tmp_traj_point.pose.position.x = trajectory_position_data[i + nearestIndex, 0]
                tmp_traj_point.pose.position.y = trajectory_position_data[i + nearestIndex, 1]
                tmp_traj_point.pose.position.z = present_position[2]

                tmp_traj_point.pose.orientation.x = 0.0
                tmp_traj_point.pose.orientation.y = 0.0
                tmp_traj_point.pose.orientation.z = np.sin(
                    trajectory_yaw_data[i + nearestIndex] / 2
                )
                tmp_traj_point.pose.orientation.w = np.cos(
                    trajectory_yaw_data[i + nearestIndex] / 2
                )

                tmp_traj_point.longitudinal_velocity_mps = trajectory_longitudinal_velocity_data[
                    i + nearestIndex
                ]
                tmp_traj.points.append(tmp_traj_point)

            self.trajectory_for_collecting_data_pub_.publish(tmp_traj)

            # [4] publish marker_array
            marker_array = MarkerArray()

            # [4a] local trajectory
            marker_traj1 = Marker()
            marker_traj1.type = 4
            marker_traj1.id = 1
            marker_traj1.header.frame_id = "map"

            marker_traj1.action = marker_traj1.ADD

            marker_traj1.scale.x = 0.3
            marker_traj1.scale.y = 0.0
            marker_traj1.scale.z = 0.0

            marker_traj1.color.a = 1.0
            marker_traj1.color.r = 1.0
            marker_traj1.color.g = 0.0
            marker_traj1.color.b = 0.0

            marker_traj1.lifetime.nanosec = 500000000
            marker_traj1.frame_locked = True

            marker_traj1.points = []
            for i in range(len(tmp_traj.points)):
                tmp_marker_point = Point()
                tmp_marker_point.x = tmp_traj.points[i].pose.position.x
                tmp_marker_point.y = tmp_traj.points[i].pose.position.y
                tmp_marker_point.z = 0.0
                marker_traj1.points.append(tmp_marker_point)

            marker_array.markers.append(marker_traj1)

            # [4b] whole trajectory
            marker_traj2 = Marker()
            marker_traj2.type = 4
            marker_traj2.id = 0
            marker_traj2.header.frame_id = "map"

            marker_traj2.action = marker_traj2.ADD

            marker_traj2.scale.x = 0.3
            marker_traj2.scale.y = 0.0
            marker_traj2.scale.z = 0.0

            marker_traj2.color.a = 1.0
            marker_traj2.color.r = 0.0
            marker_traj2.color.g = 0.0
            marker_traj2.color.b = 1.0

            marker_traj2.lifetime.nanosec = 500000000
            marker_traj2.frame_locked = True

            marker_traj2.points = []
            marker_downsampling = 5
            for i in range((len(trajectory_position_data) // marker_downsampling)):
                tmp_marker_point = Point()
                tmp_marker_point.x = trajectory_position_data[i * marker_downsampling, 0]
                tmp_marker_point.y = trajectory_position_data[i * marker_downsampling, 1]
                tmp_marker_point.z = 0.0
                marker_traj2.points.append(tmp_marker_point)

            marker_array.markers.append(marker_traj2)

            self.data_collecting_trajectory_marker_array_pub_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    data_collecting_trajectory_publisher = DataCollectingTrajectoryPublisher()

    rclpy.spin(data_collecting_trajectory_publisher)

    data_collecting_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
