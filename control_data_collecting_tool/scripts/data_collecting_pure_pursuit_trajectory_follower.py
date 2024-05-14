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
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


def getYaw(orientation_xyzw):
    return R.from_quat(orientation_xyzw.reshape(-1, 4)).as_euler("xyz")[:, 2]


def linearized_pure_pursuit_control(
    pos_xy_obs,
    pos_yaw_obs,
    longitudinal_vel_obs,
    pos_xy_ref_nearest,
    pos_yaw_ref_nearest,
    longitudinal_vel_ref_nearest,
):
    # control law equal to simple_trajectory_follower in autoware
    wheel_base = 4.0
    pure_pursuit_acc_kp = 0.5
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


# def naive_pure_pursuit_control(
#     pos_xy_obs, pos_yaw_obs, longitudinal_vel_obs, pos_xy_ref_nearest, pos_yaw_ref_nearest, longitudinal_vel_ref_nearest
# ):
#     linx = longitudinal_vel_ref_nearest
#     angz = 2.0 * linx * np.sin(alpha)/L
#     steer = angz_to_steer(linx, angz)
#     acc = - 0.1 * (longitudinal_vel_obs - longitudinal_vel_ref_nearest)
#     return np.array([acc, steer])


class DataCollectingPurePursuitTrajetoryFollower(Node):
    def __init__(self):
        super().__init__("data_collecting_pure_pursuit_trajectory_follower")

        self.sub_odometry_ = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.onOdometry,
            1,
        )
        self.sub_odometry_

        self.sub_trajectory_ = self.create_subscription(
            Trajectory,
            # "/planning/scenario_planning/trajectory",
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

        self.timer_period_callback = 0.03  # 30ms
        self.timer = self.create_timer(self.timer_period_callback, self.timer_callback)

        self._present_kinematic_state = None
        self._present_trajectory = None

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onTrajectory(self, msg):
        self._present_trajectory = msg

    def timer_callback(self):
        if (self._present_trajectory is not None) and (self._present_kinematic_state is not None):
            self.control()

    def control(self):
        # [1] receive topic
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
        ave_step = np.sqrt(
            ((trajectory_position[1:, :2] - trajectory_position[:-1, :2]) ** 2).sum(axis=1)
        ).mean()
        self.get_logger().info('present_yaw: "%s"' % present_yaw)

        # [2] compute control
        # [2b] naive pure pursuit
        # self.get_logger().info('ave_step: "%s"' % ave_step)
        distance = np.sqrt(((trajectory_position - present_position) ** 2).sum(axis=1))

        # 位置が近い順に並べる
        index_array_near = np.argsort(distance)
        nearestIndex = index_array_near[0]

        max_cos_diff_yaw_value = -1
        nearestIndex = None
        for i in range(len(index_array_near)):
            if (distance[index_array_near[0]] + ave_step * 5) < distance[index_array_near[i]]:
                if nearestIndex is None:
                    nearestIndex = index_array_near[0]
                break
            tmp_cos_diff_yaw_value = np.cos(
                getYaw(trajectory_orientation[index_array_near[i]]) - present_yaw
            )
            if tmp_cos_diff_yaw_value >= max_cos_diff_yaw_value:
                max_cos_diff_yaw_value = 1.0 * tmp_cos_diff_yaw_value
                nearestIndex = 1 * index_array_near[i]

        self.get_logger().info('distance: "%s"' % distance[index_array_near[:10]])

        """
        # 一番位置が近いものの周辺で最も姿勢が近いものを選ぶ
        lookahead_length = 5.0
        lookahead_indices = 2 * int(lookahead_length/ave_step)
        max_cos_diff_yaw_value = -1
        nearestIndex = None
        for i in range(len(index_array_near)):
            if (distance[index_array_near[0]]+ave_step*5) < distance[index_array_near[i]]:
                if nearestIndex is None:
                    nearestIndex = index_array_near[0]
                break
            tmp_cos_diff_yaw_value = np.cos( getYaw(trajectory_orientation[index_array_near[i]]) - present_yaw )
            if (tmp_cos_diff_yaw_value >= max_cos_diff_yaw_value) and ((lookahead_indices+index_array_near[i])<len(index_array_near)):
                max_cos_diff_yaw_value = 1.0 * tmp_cos_diff_yaw_value
                nearestIndex = 1 * index_array_near[i]

        # ターゲットを選ぶ
        targetIndex = 1 * nearestIndex
        while True:
            tmp_distance = np.sqrt(((trajectory_position[targetIndex] - present_position) ** 2).sum())
            if tmp_distance>lookahead_length:
                break
            targetIndex += 1
        """
        target_yaw = getYaw(trajectory_orientation[nearestIndex])
        self.get_logger().info('target_yaw: "%s"' % target_yaw)

        # for i in range(len(index_array_near)):
        #     nearestIndex = index_array_near[i]
        #     if (index_array_near[i]+lookahead_indices)<len(index_array_near):
        #         break
        #     else:
        #         self.get_logger().info('yes')

        # for i in range(len(index_array)):

        # target_index = index_array[0]
        #     target_direction_yaw = getYaw(trajectory_orientation[index_array[i]])
        #     dot_product = np.cos(target_direction_yaw)*np.cos(present_yaw) + np.sin(target_direction_yaw)*np.sin(present_yaw)
        #     if dot_product > 0.0:
        #         target_index = 1 * i
        #         self.get_logger().info('index_array: "%s"' % index_array)
        #         self.get_logger().info('i: "%s"' % i)
        #         break
        # cmd = np.zeros(2)  # dummy

        # [2a] linearized pure pursuit
        # nearestIndex = ((trajectory_position - present_position) ** 2).sum(axis=1).argmin()
        closest_traj_position = trajectory_position[nearestIndex]
        closest_traj_yaw = getYaw(trajectory_orientation[nearestIndex])
        closest_traj_longitudinal_velocity = trajectory_longitudinal_velocity[nearestIndex]
        cmd = linearized_pure_pursuit_control(
            present_position[:2],
            present_yaw,
            present_linear_velocity[0],
            closest_traj_position[:2],
            closest_traj_yaw,
            closest_traj_longitudinal_velocity,
        )

        control_cmd_msg = AckermannControlCommand()
        control_cmd_msg.stamp = (
            control_cmd_msg.lateral.stamp
        ) = control_cmd_msg.longitudinal.stamp = (self.get_clock().now().to_msg())
        control_cmd_msg.longitudinal.speed = closest_traj_longitudinal_velocity
        control_cmd_msg.longitudinal.acceleration = cmd[0]
        control_cmd_msg.lateral.steering_tire_angle = cmd[1]

        self.control_cmd_pub_.publish(control_cmd_msg)

        gear_cmd_msg = GearCommand()
        gear_cmd_msg.stamp = control_cmd_msg.lateral.stamp
        gear_cmd_msg.command = GearCommand.DRIVE
        self.gear_cmd_pub_.publish(gear_cmd_msg)


def main(args=None):
    rclpy.init(args=args)

    data_collecting_pure_pursuit_trajectory_follower = DataCollectingPurePursuitTrajetoryFollower()

    rclpy.spin(data_collecting_pure_pursuit_trajectory_follower)

    data_collecting_pure_pursuit_trajectory_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
