import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.srv import GetMap, GetMap_Response
from nav_msgs.msg import OccupancyGrid 
import runner_logic

class MazeRunnerNode(Node):
    def __init__(self):
        super().__init__("maze_runner")

        self.get_logger().info("Fetching map...")
        self.map_client = self.create_client(GetMap, 'map_server/map')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Wating for map server to become available...")

        response: GetMap_Response = self.map_client.call(GetMap.Request()) # type: ignore # the generic here doesn't work properly
        self.map: OccupancyGrid = response.map

        self.initial_pose: Pose | None = None
        self.goal_pose: Pose | None = None

        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.set_initial_pose, 10)
        self.goal_pose_sub = self.create_subscription(Pose, 'goal_pose', self.set_goal_pose, 10)

        self.get_logger().info("Map fetched, waiting for initial and goal poses...")

        # setup for later
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.planned_trajectory: list[Pose] = []

        timer_period = 0.1
        self.path_follow_timer = self.create_timer(timer_period, self.run_loop, autostart=False)

    def set_initial_pose(self, msg: PoseWithCovarianceStamped):
        self.initial_pose = msg.pose.pose
        if self.goal_pose is not None:
            self.get_logger().info("Initial and goal poses received, planning trajectory...")
            self.plan_trajectory()

    def set_goal_pose(self, msg: Pose):
        self.goal_pose = msg
        if self.initial_pose is not None:
            self.get_logger().info("Initial and goal poses received, planning trajectory...")
            self.plan_trajectory()

    def plan_trajectory(self):
        self.planned_trajectory = runner_logic.plan_trajectory(self.map, self.initial_pose, self.goal_pose)
        self.path_follow_timer.reset() # start navigation

    def run_loop(self):
        # TODO
        pass
    

def main():
    rclpy.init()
    node = MazeRunnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
