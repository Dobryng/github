import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from control_msgs.action import GripperCommand
from tf2_ros import Buffer, TransformListener, LookupException
import tf2_geometry_msgs


import threading

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # 订阅物体位置
        self.subscription = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',
            self.object_pose_callback,
            10)

        # 初始化 TF 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始化夹爪的 Action Client
        self.gripper_client = ActionClient(self, GripperCommand, '/jetarm_gripper_controller/gripper_cmd')

        # 初始化 MoveGroup 的 Action Client
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        # 标记是否正在执行抓取操作，避免重复执行
        self.is_busy = False

    def object_pose_callback(self, msg):
        if self.is_busy:
            return  # 忙碌中，忽略新的物体位置

        self.is_busy = True

        # 在新线程中执行抓取任务，避免阻塞回调
        threading.Thread(target=self.process_object_pose, args=(msg,)).start()

    def process_object_pose(self, msg):
        # 将物体位置从相机坐标系转换到机械臂基座坐标系
        try:
            trans = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
            object_pose_base = tf2_geometry_msgs.do_transform_pose(msg, trans)
        except LookupException as e:
            self.get_logger().error(f'坐标转换失败：{e}')
            self.is_busy = False
            return

        # 控制机械臂进行抓取和放置
        self.pick_and_place(object_pose_base)

        self.is_busy = False

    def pick_and_place(self, pose_stamped):
        # 移动到预抓取位置（稍微高于物体）
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = 'base_link'
        pre_grasp_pose.pose.position = pose_stamped.pose.position
        pre_grasp_pose.pose.position.z += 0.1  # 提高 10 cm
        pre_grasp_pose.pose.orientation.w = 1.0  # 根据需要设置

        # 移动到预抓取位置
        self.move_to_pose(pre_grasp_pose)

        # 移动到抓取位置
        self.move_to_pose(pose_stamped)

        # 闭合夹爪
        self.control_gripper(position=0.0)  # 0.0 表示闭合

        # 提起物体
        self.move_to_pose(pre_grasp_pose)

        # 移动到放置位置
        place_pose = PoseStamped()
        place_pose.header.frame_id = 'base_link'
        place_pose.pose.position.x = 0.5
        place_pose.pose.position.y = 0.0
        place_pose.pose.position.z = 0.2
        place_pose.pose.orientation.w = 1.0  # 根据需要设置

        self.move_to_pose(place_pose)

        # 打开夹爪
        self.control_gripper(position=1.0)  # 1.0 表示完全打开

        # 返回初始位置
        home_pose = PoseStamped()
        home_pose.header.frame_id = 'base_link'
        home_pose.pose.position.x = 0.0
        home_pose.pose.position.y = 0.0
        home_pose.pose.position.z = 0.5  # 根据您的机械臂初始高度
        home_pose.pose.orientation.w = 1.0

        self.move_to_pose(home_pose)

    def move_to_pose(self, target_pose):
        # 构建 MoveGroup 目标
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm_group'
        goal_msg.request.goal_constraints.append(self.pose_to_constraints(target_pose))
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # 发送目标并等待结果
        self.move_group_client.wait_for_server()
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('运动目标被拒绝')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.error_code.val != result.error_code.SUCCESS:
            self.get_logger().error(f'运动规划失败，错误代码：{result.error_code.val}')

    def pose_to_constraints(self, pose_stamped):
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
        from shape_msgs.msg import SolidPrimitive

        constraints = Constraints()

        # 位置约束
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = 'gripper_link'  # 您的末端执行器链接名称
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # 定义约束区域
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 允许的误差范围

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(pose_stamped.pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)

        # 方向约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = 'gripper_link'  # 您的末端执行器链接名称
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def control_gripper(self, position):
        # 构建夹爪控制目标
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position  # 0.0（闭合）到 1.0（打开）

        # 发送目标并等待结果
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('夹爪目标被拒绝')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status != 0:  # 0 表示成功
            self.get_logger().error('夹爪动作失败')

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被手动中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
