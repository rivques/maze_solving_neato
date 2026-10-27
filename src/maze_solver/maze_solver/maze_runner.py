from maze_solver.angle_helpers import pose_to_tuple
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, Point, PointStamped, PoseStamped
from nav_msgs.srv import GetMap, GetMap_Response
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from .runner_logic import plan_trajectory
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformStamped

class MazeRunnerNode(Node):
    def __init__(self):
        super().__init__("maze_runner")

        self.get_logger().info("Fetching map...")
        self.map_client = self.create_client(GetMap, 'map_server/map')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for map server to become available...")
        self.get_logger().info("Map server available, requesting map...")
        future = self.map_client.call_async(GetMap.Request()) # the synchronous call doesn't work for some reason
        rclpy.spin_until_future_complete(self, future)
        response: GetMap_Response = future.result() # pyright: ignore[reportAssignmentType]
        self.map: OccupancyGrid = response.map

        self.initial_pose: Pose | None = None
        self.goal_pose: Pose | None = None

        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.set_initial_pose, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, 'goal_pose', self.set_goal_pose, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.broadcast_tf)
        self.tf = None
        self.get_logger().info("Map fetched, waiting for initial and goal poses...")

        # setup for later
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.planned_trajectory: list[Pose] = []
        self.executed_trajectory: list[PoseStamped] = []

        # debug publishers
        self.target_point_pub = self.create_publisher(PointStamped, 'target_point', 10)
        self.closest_point_pub = self.create_publisher(PointStamped, 'closest_point', 10)
        self.planned_path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.executed_path_pub = self.create_publisher(Path, 'executed_path', 10)
        # parameters
        self.declare_parameter("target_dist_forward", 0.1)
        self.declare_parameter("linear_kp", 1)
        self.declare_parameter("angular_kp", 2)

    def broadcast_tf(self):
        if self.tf is not None:
            self.tf.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.tf)

    def set_initial_pose(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info(f"Received initial pose: {msg.pose.pose}")
        self.initial_pose = msg.pose.pose

        # broadcast a new tf for map->odom
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = self.initial_pose.position.x
        transform.transform.translation.y = self.initial_pose.position.y
        transform.transform.translation.z = self.initial_pose.position.z
        transform.transform.rotation = self.initial_pose.orientation
        self.tf = transform

        if self.goal_pose is not None:
            self.get_logger().info("Initial and goal poses received, planning trajectory...")
            self.plan_trajectory()

    def set_goal_pose(self, msg: PoseStamped):
        self.get_logger().info(f"Received goal pose: {msg.pose}")
        self.goal_pose = msg.pose
        if self.initial_pose is not None:
            self.get_logger().info("Initial and goal poses received, planning trajectory...")
            self.plan_trajectory()

    def plan_trajectory(self):
        self.planned_trajectory = plan_trajectory(self.map, self.initial_pose, self.goal_pose) # pyright: ignore[reportArgumentType]
        self.get_logger().info(f"Planned trajectory with {len(self.planned_trajectory)} poses")
        # convert the list of poses to PoseStamped for Path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.poses = [PoseStamped(pose=pose) for pose in self.planned_trajectory]
        self.planned_path_pub.publish(path_msg)
        
    def odom_callback(self, msg: Odometry):
        if len(self.planned_trajectory) == 0:
            return # no trajectory to follow yet
        
        # path following algorithm:
        # figure out the closest point to us on the path
        # target a point a configurable distance along the path
        # p controllers for angular and linear velocity
        # as we move along the path, so too does the target point
        # i _think_ this is called pure pursuit, but idr

        x_odom, y_odom, theta_odom = pose_to_tuple(msg.pose.pose)
        # convert to map frame with homogeneous transform
        pose_matrix_odom = np.array([[np.cos(theta_odom), -np.sin(theta_odom), x_odom],
                                     [np.sin(theta_odom), np.cos(theta_odom), y_odom],
                                     [0, 0, 1]])
        x_init_map, y_init_map, theta_init_map = pose_to_tuple(self.initial_pose) # pyright: ignore[reportArgumentType]
        init_matrix_map = np.array([[np.cos(theta_init_map), -np.sin(theta_init_map), x_init_map],
                                         [np.sin(theta_init_map), np.cos(theta_init_map), y_init_map],
                                         [0, 0, 1]])
        pose_matrix_map = init_matrix_map @ pose_matrix_odom
        x_map, y_map = pose_matrix_map[0, 2], pose_matrix_map[1, 2]
        theta_map = np.arctan2(pose_matrix_map[1, 0], pose_matrix_map[0, 0])
    
        self.executed_trajectory.append(PoseStamped(pose=msg.pose.pose, header=msg.header))
        executed_path_msg = Path()
        executed_path_msg.header.frame_id = "odom"
        executed_path_msg.poses = self.executed_trajectory
        self.executed_path_pub.publish(executed_path_msg)

        path_points = np.array([[pose.position.x, pose.position.y] for pose in self.planned_trajectory])
        odom_point = np.array([x_map, y_map])
        distances = np.linalg.norm(path_points - odom_point, axis=1)
        closest_index = np.argmin(distances)
        
        target_index = closest_index
        for i in range(closest_index, len(self.planned_trajectory)):
            if np.linalg.norm(path_points[i] - odom_point) > self.get_parameter("target_dist_forward").value:
                target_index = i
                break
        else:
            target_index = len(self.planned_trajectory) - 1
        
        if target_index == closest_index:
            self.get_logger().info("Reached the end of the trajectory")
            self.cmd_vel_pub.publish(Twist()) # stop the robot
            self.planned_trajectory = []
            return

        target_point = path_points[target_index]
        closest_point = path_points[closest_index]
        target_point_msg = PointStamped()
        target_point_msg.header.frame_id = "map"
        target_point_msg.point.x = target_point[0]
        target_point_msg.point.y = target_point[1]
        closest_point_msg = PointStamped()
        closest_point_msg.header.frame_id = "map"
        closest_point_msg.point.x = closest_point[0]
        closest_point_msg.point.y = closest_point[1]
        self.target_point_pub.publish(target_point_msg)
        self.closest_point_pub.publish(closest_point_msg)

        # take the target point back to odom frame
        target_matrix_map = np.array([[np.cos(theta_init_map), -np.sin(theta_init_map), target_point[0]],
                                      [np.sin(theta_init_map), np.cos(theta_init_map), target_point[1]],
                                      [0, 0, 1]])
        target_matrix_odom = np.linalg.inv(init_matrix_map) @ target_matrix_map
        target_x_odom, target_y_odom = target_matrix_odom[0, 2], target_matrix_odom[1, 2]

        # controller time
        vector_between = np.array([target_x_odom, target_y_odom]) - np.array([x_odom, y_odom])
        linear_error = np.linalg.norm(vector_between)
        angular_error = np.arctan2(vector_between[1], vector_between[0]) - theta_odom
        # wrap angular error to [-pi, pi]
        angular_error = (angular_error + np.pi) % (2 * np.pi) - np.pi

        linear = self.get_parameter("linear_kp").value * linear_error
        angular = self.get_parameter("angular_kp").value * angular_error
        print(f"v_l: {linear:.2f}, v_a: {angular:.2f}")
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear
        cmd_vel_msg.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
def main():
    rclpy.init()
    node = MazeRunnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
