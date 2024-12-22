# src/grasp_trigger.py

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from rclpy.action import ActionClient

class GraspTrigger(Node):
    def __init__(self):
        super().__init__('grasp_trigger')
        self.subscription = self.create_subscription(
            Detection2DArray,
            'detections_output',
            self.detections_callback,
            10)
        self.move_client = ActionClient(self, FollowJointTrajectory, '/jetarm_arm_controller/follow_joint_trajectory')
        self.grasping = False

    def detections_callback(self, msg):
        if self.grasping:
            return

        for detection in msg.detections:
            class_id = int(detection.results[0].hypothesis.class_id)
            if class_id == 39:  # 'bottle'
                self.grasping = True
                # 計算抓取位置
                pose = self.calculate_pose(detection)
                self.plan_and_execute(pose)
                break

    def calculate_pose(self, detection):
        # 根據檢測結果計算物體在機械臂坐標系中的位置
        # 這需要相機的內參和 TF 轉換
        # 這裡假設已經有物體的 3D 座標
        pose = Pose()
        pose.position.x = 0.5  # 示例值
        pose.position.y = 0.0
        pose.position.z = 0.2
        pose.orientation.w = 1.0
        return pose

    def plan_and_execute(self, pose):
        # 使用 MoveIt2 的 MoveGroup 來規劃運動
        # 此處需要實現 MoveGroup 的調用
        # 示例簡化為直接發送關節軌跡
        goal_msg = FollowJointTrajectory.Goal()
        # 設置目標軌跡點
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # 示例值，需要根據姿態計算
        point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self.move_client.wait_for_server()
        self.send_goal_future = self.move_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.grasping = False
            return
        self.get_logger().info('Goal accepted')
        self.get_logger().info('Executing trajectory...')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback')

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed with status {}'.format(status))
        self.grasping = False

def main(args=None):
    rclpy.init(args=args)
    node = GraspTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
