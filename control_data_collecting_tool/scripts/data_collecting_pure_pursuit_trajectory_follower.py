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

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_vehicle_msgs.msg import GearCommand
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

debug_matplotlib_plot_flag = False
if debug_matplotlib_plot_flag:
    import matplotlib.pyplot as plt

    plt.rcParams["figure.figsize"] = [8, 8]


def getYaw(orientation_xyzw):
    return R.from_quat(orientation_xyzw.reshape(-1, 4)).as_euler("xyz")[:, 2]


class DataCollectingPurePursuitTrajetoryFollower(Node):
    def __init__(self):
        super().__init__("data_collecting_pure_pursuit_trajectory_follower")

        self.declare_parameter(
            "wheel_base",
            2.79,  # sample_vehicle_launch/sample_vehicle_description/config/vehicle_info.param.yaml
            ParameterDescriptor(description="Wheel base [m]"),
        )

        self.declare_parameter(
            "pure_pursuit_acc_kp",
            0.5,
            ParameterDescriptor(description="Pure pursuit accel command propotional gain"),
        )

        self.declare_parameter(
            "naive_pure_pursuit_lookahead_length",
            10.0,
            ParameterDescriptor(description="Pure pursuit (naive version) lookahead length [m]"),
        )

        self.declare_parameter(
            "acc_noise_amp",
            0.05,
            ParameterDescriptor(description="Accel cmd additional sine noise amplitude [m/ss]"),
        )

        self.declare_parameter(
            "acc_noise_min_period",
            5.0,
            ParameterDescriptor(description="Accel cmd additional sine noise minimum period [s]"),
        )

        self.declare_parameter(
            "acc_noise_max_period",
            20.0,
            ParameterDescriptor(description="Accel cmd additional sine noise maximum period [s]"),
        )

        self.declare_parameter(
            "steer_noise_amp",
            0.01,
            ParameterDescriptor(description="Steer cmd additional sine noise amplitude [rad]"),
        )

        self.declare_parameter(
            "steer_noise_min_period",
            5.0,
            ParameterDescriptor(description="Steer cmd additional sine noise minimum period [s]"),
        )

        self.declare_parameter(
            "steer_noise_max_period",
            20.0,
            ParameterDescriptor(description="Steer cmd additional sine noise maximum period [s]"),
        )

        self.sub_odometry_ = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.onOdometry,
            1,
        )
        self.sub_odometry_

        self.sub_trajectory_ = self.create_subscription(
            Trajectory,
            "/data_collecting_trajectory",
            self.onTrajectory,
            1,
        )
        self.sub_trajectory_

        self.control_cmd_pub_ = self.create_publisher(
            AckermannControlCommand,
            "/external/selected/control_cmd",
            1,
        )

        self.gear_cmd_pub_ = self.create_publisher(
            GearCommand,
            "/external/selected/gear_cmd",
            1,
        )

        self.data_collecting_lookahead_marker_array_pub_ = self.create_publisher(
            MarkerArray,
            "/data_collecting_lookahead_marker_array",
            1,
        )

        self.timer_period_callback = 0.03  # 30ms
        self.timer = self.create_timer(self.timer_period_callback, self.timer_callback)

        self._present_kinematic_state = None
        self._present_trajectory = None

        self.acc_noise_list = []
        self.steer_noise_list = []
        self.acc_history = []
        self.steer_history = []
        self.acc_noise_history = []
        self.steer_noise_history = []

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onTrajectory(self, msg):
        self._present_trajectory = msg

    def timer_callback(self):
        if (self._present_trajectory is not None) and (self._present_kinematic_state is not None):
            self.control()

    def linearized_pure_pursuit_control(
        self,
        pos_xy_obs,
        pos_yaw_obs,
        longitudinal_vel_obs,
        pos_xy_ref_nearest,
        pos_yaw_ref_nearest,
        longitudinal_vel_ref_nearest,
    ):
        # control law equal to simple_trajectory_follower in autoware
        wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        pure_pursuit_acc_kp = (
            self.get_parameter("pure_pursuit_acc_kp").get_parameter_value().double_value
        )

        # Currently, the following params are not declareed as ROS 2 params.
        pure_pursuit_lookahead_time = 3.0
        pure_pursuit_min_lookahead = 3.0
        pure_pursuit_steer_kp_param = 2.0
        pure_pursuit_steer_kd_param = 2.0

        longitudinal_vel_err = longitudinal_vel_obs - longitudinal_vel_ref_nearest
        pure_pursuit_acc_cmd = -pure_pursuit_acc_kp * longitudinal_vel_err

        cos_yaw = np.cos(pos_yaw_ref_nearest)
        sin_yaw = np.sin(pos_yaw_ref_nearest)
        diff_position = pos_xy_obs - pos_xy_ref_nearest
        lat_err = -sin_yaw * diff_position[0] + cos_yaw * diff_position[1]
        yaw_err = pos_yaw_obs - pos_yaw_ref_nearest
        lat_err = np.array([lat_err]).flatten()[0]
        yaw_err = np.array([yaw_err]).flatten()[0]
        while True:
            if yaw_err > np.pi:
                yaw_err -= 2.0 * np.pi
            if yaw_err < (-np.pi):
                yaw_err += 2.0 * np.pi
            if np.abs(yaw_err) < np.pi:
                break

        lookahead = pure_pursuit_min_lookahead + pure_pursuit_lookahead_time * np.abs(
            longitudinal_vel_obs
        )
        pure_pursuit_steer_kp = pure_pursuit_steer_kp_param * wheel_base / (lookahead * lookahead)
        pure_pursuit_steer_kd = pure_pursuit_steer_kd_param * wheel_base / lookahead
        pure_pursuit_steer_cmd = -pure_pursuit_steer_kp * lat_err - pure_pursuit_steer_kd * yaw_err
        return np.array([pure_pursuit_acc_cmd, pure_pursuit_steer_cmd])

    def naive_pure_pursuit_control(
        self,
        pos_xy_obs,
        pos_yaw_obs,
        longitudinal_vel_obs,
        pos_xy_ref_target,
        longitudinal_vel_ref_nearest,
    ):
        wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        pure_pursuit_acc_kp = (
            self.get_parameter("pure_pursuit_acc_kp").get_parameter_value().double_value
        )
        longitudinal_vel_err = longitudinal_vel_obs - longitudinal_vel_ref_nearest
        pure_pursuit_acc_cmd = -pure_pursuit_acc_kp * longitudinal_vel_err

        rot_matrix = np.array(
            [
                [np.cos(pos_yaw_obs), np.sin(pos_yaw_obs)],
                [-np.sin(pos_yaw_obs), np.cos(pos_yaw_obs)],
            ]
        ).reshape(2, 2)
        target_position_from_vehicle = rot_matrix @ (pos_xy_ref_target - pos_xy_obs[:2])
        alpha = np.arctan(target_position_from_vehicle[1] / target_position_from_vehicle[0])
        angz = 2.0 * longitudinal_vel_ref_nearest * np.sin(alpha) / wheel_base
        steer = np.arctan(angz * wheel_base / longitudinal_vel_ref_nearest)
        return np.array([pure_pursuit_acc_cmd, steer])

    def control(self):
        # [0] receive topic
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
        present_yaw = getYaw(present_orientation)

        trajectory_position = []
        trajectory_orientation = []
        trajectory_longitudinal_velocity = []
        points = self._present_trajectory.points
        for i in range(len(points)):
            trajectory_position.append(
                [points[i].pose.position.x, points[i].pose.position.y, points[i].pose.position.z]
            )
            trajectory_orientation.append(
                [
                    points[i].pose.orientation.x,
                    points[i].pose.orientation.y,
                    points[i].pose.orientation.z,
                    points[i].pose.orientation.w,
                ]
            )
            trajectory_longitudinal_velocity.append(points[i].longitudinal_velocity_mps)
        trajectory_position = np.array(trajectory_position)
        trajectory_orientation = np.array(trajectory_orientation)
        trajectory_longitudinal_velocity = np.array(trajectory_longitudinal_velocity)

        nearestIndex = (
            ((trajectory_position[:, :2] - present_position[:2]) ** 2).sum(axis=1).argmin()
        )

        # prepare noise
        if len(self.acc_noise_list) == 0:
            tmp_noise_amp = (
                np.random.rand()
                * self.get_parameter("acc_noise_amp").get_parameter_value().double_value
            )
            noise_min_period = (
                self.get_parameter("acc_noise_min_period").get_parameter_value().double_value
            )
            noise_max_period = (
                self.get_parameter("acc_noise_max_period").get_parameter_value().double_value
            )
            tmp_noise_period = noise_min_period + np.random.rand() * (
                noise_max_period - noise_min_period
            )
            dt = self.timer_period_callback
            noise_data_num = max(4, int(tmp_noise_period / dt))
            for i in range(noise_data_num):
                self.acc_noise_list.append(tmp_noise_amp * np.sin(2.0 * np.pi * i / noise_data_num))
        if len(self.steer_noise_list) == 0:
            tmp_noise_amp = (
                np.random.rand()
                * self.get_parameter("steer_noise_amp").get_parameter_value().double_value
            )
            noise_min_period = (
                self.get_parameter("steer_noise_min_period").get_parameter_value().double_value
            )
            noise_max_period = (
                self.get_parameter("steer_noise_max_period").get_parameter_value().double_value
            )
            tmp_noise_period = noise_min_period + np.random.rand() * (
                noise_max_period - noise_min_period
            )

            dt = self.timer_period_callback
            noise_data_num = max(4, int(tmp_noise_period / dt))
            for i in range(noise_data_num):
                self.steer_noise_list.append(
                    tmp_noise_amp * np.sin(2.0 * np.pi * i / noise_data_num)
                )

        # [1] compute control
        controller_name = "naive_pure_pursuit_control"
        cmd = np.zeros(2)

        # [1a] linearized pure pursuit (deprecated)
        # closest_traj_position = trajectory_position[nearestIndex]
        # closest_traj_yaw = getYaw(trajectory_orientation[nearestIndex])
        # closest_traj_longitudinal_velocity = trajectory_longitudinal_velocity[nearestIndex]
        # cmd = self.linearized_pure_pursuit_control(
        #     present_position[:2],
        #     present_yaw,
        #     present_linear_velocity[0],
        #     closest_traj_position[:2],
        #     closest_traj_yaw,
        #     closest_traj_longitudinal_velocity,
        # )

        # [1b] naive pure pursuit
        if controller_name == "naive_pure_pursuit_control":
            targetIndex = 1 * nearestIndex
            naive_pure_pursuit_lookahead_length = (
                self.get_parameter("naive_pure_pursuit_lookahead_length")
                .get_parameter_value()
                .double_value
            )
            while True:
                tmp_distance = np.sqrt(
                    ((trajectory_position[targetIndex][:2] - present_position[:2]) ** 2).sum()
                )
                if tmp_distance > naive_pure_pursuit_lookahead_length:
                    break
                if targetIndex == (len(trajectory_position) - 1):
                    break
                targetIndex += 1

            cmd = self.naive_pure_pursuit_control(
                present_position[:2],
                present_yaw,
                present_linear_velocity[0],
                trajectory_position[targetIndex][:2],
                trajectory_longitudinal_velocity[nearestIndex],
            )

        cmd_without_noise = 1 * cmd

        tmp_acc_noise = self.acc_noise_list.pop(0)
        tmp_steer_noise = self.steer_noise_list.pop(0)

        cmd[0] += tmp_acc_noise
        cmd[1] += tmp_steer_noise

        # [2] publish cmd
        control_cmd_msg = AckermannControlCommand()
        control_cmd_msg.stamp = (
            control_cmd_msg.lateral.stamp
        ) = control_cmd_msg.longitudinal.stamp = (self.get_clock().now().to_msg())
        control_cmd_msg.longitudinal.speed = trajectory_longitudinal_velocity[nearestIndex]
        control_cmd_msg.longitudinal.acceleration = cmd[0]
        control_cmd_msg.lateral.steering_tire_angle = cmd[1]

        self.control_cmd_pub_.publish(control_cmd_msg)

        gear_cmd_msg = GearCommand()
        gear_cmd_msg.stamp = control_cmd_msg.lateral.stamp
        gear_cmd_msg.command = GearCommand.DRIVE
        self.gear_cmd_pub_.publish(gear_cmd_msg)

        # [3] publish marker
        if controller_name == "naive_pure_pursuit_control":
            marker_array = MarkerArray()

            marker_traj = Marker()
            marker_traj.type = 4
            marker_traj.id = 1
            marker_traj.header.frame_id = "map"

            marker_traj.action = marker_traj.ADD

            marker_traj.scale.x = 0.6
            marker_traj.scale.y = 0.0
            marker_traj.scale.z = 0.0

            marker_traj.color.a = 1.0
            marker_traj.color.r = 0.0
            marker_traj.color.g = 1.0
            marker_traj.color.b = 0.0

            marker_traj.lifetime.nanosec = 500000000
            marker_traj.frame_locked = True

            marker_traj.points = []
            tmp_marker_point = Point()
            tmp_marker_point.x = present_position[0]
            tmp_marker_point.y = present_position[1]
            tmp_marker_point.z = 0.0
            marker_traj.points.append(tmp_marker_point)
            tmp_marker_point = Point()
            tmp_marker_point.x = trajectory_position[targetIndex][0]
            tmp_marker_point.y = trajectory_position[targetIndex][1]
            tmp_marker_point.z = 0.0
            marker_traj.points.append(tmp_marker_point)

            marker_array.markers.append(marker_traj)
            self.data_collecting_lookahead_marker_array_pub_.publish(marker_array)

        # [5] debug plot
        if debug_matplotlib_plot_flag:
            self.acc_history.append(1 * cmd[0])
            self.steer_history.append(1 * cmd[1])
            self.acc_noise_history.append(tmp_acc_noise)
            self.steer_noise_history.append(tmp_steer_noise)
            max_plot_len = 666
            if len(self.acc_history) > max_plot_len:
                self.acc_history.pop(0)
                self.steer_history.pop(0)
                self.acc_noise_history.pop(0)
                self.steer_noise_history.pop(0)
            dt = self.timer_period_callback
            timestamp = -dt * np.array(range(len(self.steer_history)))[::-1]
            plt.cla()
            plt.clf()
            plt.subplot(2, 1, 1)
            plt.plot(0, cmd[0], "o", label="current_acc")
            plt.plot(0, cmd_without_noise[0], "o", label="acc cmd without noise")
            plt.plot(timestamp, self.acc_history, "-", label="acc cmd history")
            plt.plot(timestamp, self.acc_noise_history, "-", label="acc noise history")
            plt.xlim([-20.5, 0.5])
            plt.ylim([-1, 3])
            plt.ylabel("acc [m/ss]")
            plt.legend()
            plt.subplot(2, 1, 2)
            plt.plot(0, cmd[1], "o", label="current_steer")
            plt.plot(0, cmd_without_noise[1], "o", label="steer without noise")
            plt.plot(timestamp, self.steer_history, "-", label="steer cmd history")
            plt.plot(timestamp, self.steer_noise_history, "-", label="steer noise history")
            plt.xlim([-20.5, 0.5])
            plt.ylim([-1.25, 1.25])
            plt.xlabel("future timestamp [s]")
            plt.ylabel("steer [rad]")
            plt.legend()
            plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)

    data_collecting_pure_pursuit_trajectory_follower = DataCollectingPurePursuitTrajetoryFollower()

    rclpy.spin(data_collecting_pure_pursuit_trajectory_follower)

    data_collecting_pure_pursuit_trajectory_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
