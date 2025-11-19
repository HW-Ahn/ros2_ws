import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor


class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')

        # 이 노드의 네임스페이스에 맞춰 액션 이름 설정
        ns = self.get_namespace().rstrip('/')
        if ns == '' or ns == '/':
            action_name = 'navigate_to_pose'
        else:
            action_name = f'{ns}/navigate_to_pose'

        self.get_logger().info(f'Using NavigateToPose action server: {action_name}')

        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self._sub = self.create_subscription(PoseStamped, 'cmd_goal', self.goal_cb, 10)
        self._current_goal_handle = None

        self.get_logger().info('GoalManager ready, waiting for cmd_goal msgs...')

    def goal_cb(self, msg: PoseStamped):
        self.get_logger().info(
            f'New requested goal: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}'
        )

        # 기존 goal 취소
        if self._current_goal_handle:
            self.get_logger().info('Cancelling current goal...')
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'Cancel failed: {e}')
            self._current_goal_handle = None

        # Nav2 액션 서버 준비될 때까지 충분히 기다리기
        self.get_logger().info('Waiting for NavigateToPose action server (max 20s)...')
        if not self._action_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Action server not available (timeout 20s)')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._current_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_cb(self, feedback_msg):
        # feedback_msg : NavigateToPose_FeedbackMessage
        fb = feedback_msg.feedback
        # 필요하면 여기서 진행 상황 로그 찍을 수 있음
        # self.get_logger().info(f'Progress: {fb.current_pose.pose.position.x:.2f}, {fb.current_pose.pose.position.y:.2f}')

    def get_result_callback(self, future):
        res = future.result().result
        status = future.result().status
        if status == 4:  # CANCELLED
            self.get_logger().info('Goal was cancelled')
        else:
            self.get_logger().info(f'Goal finished with status: {status}')
        self._current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
