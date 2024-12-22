import rclpy
import time
import numpy as np
from math import pi
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile
from ros_robot_controller.ros_robot_controller_sdk import Board
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import BusServoState, SetBusServoState

class RosRobotController(Node):
    def __init__(self, name):
        super().__init__(name)

        # 初始化伺服控制板
        self.board = Board()
        self.board.enable_reception()

        # 關節名稱與伺服器 ID 的對應
        self.joint_name_to_id = {
            'joint1': 1,
            'joint2': 2,
            'joint3': 3,
            'joint4': 4,
            'joint5': 5,
            'r_joint': 10,  # 夾爪伺服器的 ID
        }

        # 初始化手臂位置
        self.initialize_arm_position()

        # 建立動作服務器，監聽 FollowJointTrajectory 動作接口
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/jetarm_arm_controller/follow_joint_trajectory',
            self.execute_trajectory_callback)

        # 建立夾爪指令動作接口
        self._gripper_action_server = ActionServer(
            self,
            GripperCommand,
            '/jetarm_gripper_controller/gripper_cmd',
            self.execute_gripper_callback
        )

        # 建立關節狀態發布者
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'isaac/joint_states',
            QoSProfile(depth=10)
        )

        # 用於保存關節位置的字典
        self.current_joint_positions = {name: 0.0 for name in self.joint_name_to_id.keys()}

        # 創建定時器，用於定期發布關節狀態
        self.create_timer(0.01, self.pub_callback)  # 100 Hz

        # 日誌輸出
        self.get_logger().info('控制器已啟動，已通過動作接口進行控制')

    def shutdown(self):
        self.get_logger().info('正在關閉')
        rclpy.shutdown()

    def initialize_arm_position(self):
        duration = 5.0  # 設定移動到初始位置的時間
        initial_positions = [[1, 500], [2, 500], [3, 500], [4, 500], [5, 500], [10, 950]]  # 關節初始位置
        #initial_positions = [[1, 500], [2, 750], [3, 200], [4, 200], [5, 500], [10, 2]]  # 關節init位置        
        self.board.bus_servo_set_position(duration, initial_positions)
        self.get_logger().info('手臂已移動到初始位置')

    def apply_joint_calibration(self, positions):
        # 將伺服器的位置轉換為角度（度）
        joints = positions.copy()
        joints[0] = np.interp(joints[0], [0, 1000], [-120, 120])   # 關節1: -120 到 120 度
        joints[1] = np.interp(joints[1], [0, 1000], [-210, 30])    # 關節2: -210 到 30 度
        joints[2] = np.interp(joints[2], [0, 1000], [-120, 120])   # 關節3: -120 到 120 度
        joints[3] = np.interp(joints[3], [0, 1000], [-210, 30])    # 關節4: -210 到 30 度
        joints[4] = np.interp(joints[4], [0, 1000], [-120, 120])   # 關節5: -120 到 120 度
        joints[5] = np.interp(joints[5], [2, 950], [90, 0])        # 關節6（夾爪）: 90 到 0 度

        # 校正基準位置
        mid = np.array([0.25, -90.25, -0.45, -89.75, 0.25, 5])

        # 將 joint 值減去校正基準，得到實際的校正後值
        array = np.array(joints) - mid

        # 將校正後的角度值從度轉換為弧度
        DEG2RAD = pi / 180
        position_src = list(array * DEG2RAD)

        return position_src

    def pub_callback(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.joint_name_to_id.keys())
        positions = []
        for joint_name in joint_state_msg.name:
            joint_id = self.joint_name_to_id[joint_name]
            state = self.board.bus_servo_read_position(joint_id)

            if state is not None:
                try:
                    # 根據返回類型提取伺服器位置
                    if isinstance(state, list):
                        position = float(state[0])  # 假設列表的第一個元素是位置數據
                    else:
                        position = float(state)
                except Exception as e:
                    self.get_logger().error(f"Error processing position for joint {joint_name}: {e}")
                    position = self.current_joint_positions[joint_name]  # 使用上一個已知位置
            else:
                position = self.current_joint_positions[joint_name]  # 使用上一個已知位置

            positions.append(position)
            self.current_joint_positions[joint_name] = position  # 更新當前關節位置

        # 將伺服器位置轉換為機械臂的實際關節角度，並進行校正
        position_src = self.apply_joint_calibration(positions)
        joint_state_msg.position = position_src
        self.joint_state_publisher.publish(joint_state_msg)

    def execute_trajectory_callback(self, goal_handle):
        self.get_logger().info('接收到新的機械臂軌跡目標')

        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names

        # 檢查關節名稱是否正確
        if not set(joint_names).issubset(set(self.joint_name_to_id.keys())):
            self.get_logger().error('接收到未知的關節名稱')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        # 初始化時間
        start_time = self.get_clock().now()

        # 逐個執行軌跡點
        previous_time_from_start = 0.0
        for point in trajectory.points:
            # 計算從現在開始的目標時間
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            duration = time_from_start - previous_time_from_start
            previous_time_from_start = time_from_start

            # 將目標位置從弧度轉換為度
            RAD2DEG = 180 / pi
            positions_deg = np.array(point.positions) * RAD2DEG

            # 校正基準位置
            mid = np.array([0.25, -90.25, -0.45, -89.75, 0.25])

            # 確保關節數量匹配
            if len(positions_deg) != len(mid):
                self.get_logger().error('關節數量不匹配')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result

            # 應用校正
            positions_corrected = positions_deg + mid

            # 準備發送到伺服器的數據
            data = []
            for idx, joint_name in enumerate(joint_names):
                joint_id = self.joint_name_to_id.get(joint_name)

                if joint_id is not None:
                    angle = positions_corrected[idx]  # 使用校正後的角度

                    # 根據關節範圍映射到伺服器位置
                    if joint_name == 'joint1':
                        position = np.interp(angle, [-120, 120], [0, 1000])
                    elif joint_name == 'joint2':
                        position = np.interp(angle, [-210, 30], [0, 1000])
                    elif joint_name == 'joint3':
                        position = np.interp(angle, [-120, 120], [0, 1000])
                    elif joint_name == 'joint4':
                        position = np.interp(angle, [-210, 30], [0, 1000])
                    elif joint_name == 'joint5':
                        position = np.interp(angle, [-120, 120], [0, 1000])

                    # 確保位置是整數
                    position = int(position)
                    data.append([int(joint_id), position])

            # 發送指令
            self.board.bus_servo_set_position(duration, data)

            # 等待運動完成
            end_time = start_time + rclpy.time.Duration(seconds=time_from_start)
            sleep_duration = (end_time - self.get_clock().now()).nanoseconds / 1e9
            if sleep_duration > 0:
                time.sleep(sleep_duration)

        # 運動完成，標記目標成功
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('機械臂軌跡執行完成')
        return result



    def execute_gripper_callback(self, goal_handle):
        self.get_logger().info('接收到新的夾爪指令')

        position = goal_handle.request.command.position  # 從 GripperCommand 中獲取位置指令 (範圍 0.0 到 1.0)
        gripper_servo_id = self.joint_name_to_id.get('r_joint')

        if gripper_servo_id is not None:
            # 將夾爪的位置（0.0 到 1.0）轉換為伺服器位置範圍（2 到 950）
            gripper_position = np.interp(position, [0.0, 1.0], [950, 2])

            # 確保位置是整數
            gripper_position = int(gripper_position)

            data = [[gripper_servo_id, gripper_position]]
            self.get_logger().info(f"發送夾爪位置指令: {gripper_position} 給伺服器ID: {gripper_servo_id}")
            self.board.bus_servo_set_position(0.5, data)  # 設定一個合適的時間執行夾爪動作

            # 更新當前夾爪位置
            self.current_joint_positions['r_joint'] = gripper_position

        else:
            self.get_logger().error('找不到夾爪伺服器ID')
            goal_handle.abort()
            result = GripperCommand.Result()
            result.reached_goal = False
            return result

        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = position
        result.reached_goal = True
        result.stalled = False
        result.effort = 0.0
        self.get_logger().info('夾爪動作執行完成')
        return result


    def set_bus_servo_state(self, msg):
        data = []
        servo_id = []
        for i in msg.state:
            if i.present_id:
                if i.present_id[0]:
                    if i.target_id:
                        if i.target_id[0]:
                            self.board.bus_servo_set_id(i.present_id[1], i.target_id[1])
                    if i.position:
                        if i.position[0]:
                            data.extend([[i.present_id[1], i.position[1]]])
                    if i.offset:
                        if i.offset[0]:
                            self.board.bus_servo_set_offset(i.present_id[1], i.offset[1])
                    if i.position_limit:
                        if i.position_limit[0]:
                            self.board.bus_servo_set_angle_limit(i.present_id[1], i.position_limit[1:])
                    if i.voltage_limit:
                        if i.voltage_limit[0]:
                            self.board.bus_servo_set_vin_limit(i.present_id[1], i.voltage_limit[1:])
                    if i.max_temperature_limit:
                        if i.max_temperature_limit[0]:
                            self.board.bus_servo_set_temp_limit(i.present_id[1], i.max_temperature_limit[1])
                    if i.enable_torque:
                        if i.enable_torque[0]:
                            self.board.bus_servo_enable_torque(i.present_id[1], i.enable_torque[1])
                    if i.save_offset:
                        if i.save_offset[0]:
                            self.board.bus_servo_save_offset(i.present_id[1])
                    if i.stop:
                        if i.stop[0]:
                            servo_id.append(i.present_id[1])
        if data != []:
            self.board.bus_servo_set_position(msg.duration, data)
        if servo_id != []:
            self.board.bus_servo_stop(servo_id)

    def get_bus_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = BusServoState()
            if i.get_id:
                state = self.board.bus_servo_read_id(i.id)
                if state is not None:
                    i.id = state[0]
                    data.present_id = state
            if i.get_position:
                state = self.board.bus_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.bus_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            if i.get_temperature:
                state = self.board.bus_servo_read_temp(i.id)
                if state is not None:
                    data.temperature = state
            if i.get_position_limit:
                state = self.board.bus_servo_read_angle_limit(i.id)
                if state is not None:
                    data.position_limit = state
            if i.get_voltage_limit:
                state = self.board.bus_servo_read_vin_limit(i.id)
                if state is not None:
                    data.voltage_limit = state
            if i.get_max_temperature_limit:
                state = self.board.bus_servo_read_temp_limit(i.id)
                if state is not None:
                    data.max_temperature_limit = state
            if i.get_torque_state:
                state = self.board.bus_servo_read_torque_state(i.id)
                if state is not None:
                    data.enable_torque = state
            states.append(data)
        response.state = states
        response.success = True
        return response

def main():
    rclpy.init()
    node = RosRobotController('ros_robot_controller')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('手動中斷，正在關閉節點')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
