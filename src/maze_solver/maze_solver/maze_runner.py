from maze_solver.angle_helpers import pose_to_tuple, quaternion_from_euler
from rclpy.duration import Duration as RDuration
from rclpy.time import Time as RTime
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, Quaternion, PointStamped, PoseStamped
from nav_msgs.srv import GetMap, GetMap_Response
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from .runner_logic import plan_trajectory, get_vels_for_traj
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
        self.declare_parameter("target_dist_forward", 0.2)
        self.declare_parameter("linear_kp", 0.5)
        self.declare_parameter("angular_kp", 1)

    def broadcast_tf(self):
        if self.tf is not None:
            self.tf.header.stamp = (self.get_clock().now()).to_msg()
            self.tf_broadcaster.sendTransform(self.tf)

    def set_initial_pose(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info(f"Received initial pose: {msg.pose.pose}")
        self.initial_pose = msg.pose.pose

        # broadcast a new tf for map->odom
        current_odom = None
        try:
            current_odom = self.tf_buffer.lookup_transform("odom", "base_link", RTime(), timeout=RDuration(seconds=0))
        except Exception as e:
            # there's no odom yet, assume it's at the origin
            self.get_logger().warn(f"Could not get current odom transform: {e}, assuming odom is at the origin")
        if current_odom is not None:
            # we want to set the map->odom origin transform such that the initial pose is at the current odom
            x_init, y_init, theta_init = pose_to_tuple(self.initial_pose)
            x_odom, y_odom, theta_odom = pose_to_tuple(current_odom.transform)
            init_matrix_map = np.array([[np.cos(theta_init), -np.sin(theta_init), x_init],
                                        [np.sin(theta_init), np.cos(theta_init), y_init],
                                        [0, 0, 1]])
            odom_matrix_odom = np.array([[np.cos(theta_odom), -np.sin(theta_odom), x_odom],
                                         [np.sin(theta_odom), np.cos(theta_odom), y_odom],
                                         [0, 0, 1]])
            map_to_odom_matrix = init_matrix_map @ np.linalg.inv(odom_matrix_odom)
            x_map_to_odom, y_map_to_odom = map_to_odom_matrix[0, 2], map_to_odom_matrix[1, 2]
            theta_map_to_odom = np.arctan2(map_to_odom_matrix[1, 0], map_to_odom_matrix[0, 0])
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = "odom"
            transform.transform.translation.x = x_map_to_odom
            transform.transform.translation.y = y_map_to_odom
            transform.transform.translation.z = 0.0
            q = quaternion_from_euler(0, 0, theta_map_to_odom)
            transform.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        else:
            # assume odom is at the origin, so map->odom is just the initial pose
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

        # convert odom to map frame using most recent map->odom transform
        if self.tf is None:
            self.get_logger().warn("No map->odom transform available, cannot follow path")
            return
        x_odom, y_odom, theta_odom = pose_to_tuple(msg.pose.pose)
        x_map, y_map, theta_map = pose_to_tuple(self.tf.transform)

        traj_params = {
            "target_dist_forward": self.get_parameter("target_dist_forward").value,
            "linear_kp": self.get_parameter("linear_kp").value,
            "angular_kp": self.get_parameter("angular_kp").value,
        }
        
        linear, angular, traj_done, debug_points = get_vels_for_traj(x_odom, y_odom, theta_odom, x_map, y_map, theta_map, self.planned_trajectory, traj_params)

        # debug publishing
        target_point_msg = PointStamped()
        target_point_msg.header.frame_id = "map"
        target_point_msg.point.x = debug_points["target_point"][0]
        target_point_msg.point.y = debug_points["target_point"][1]
        closest_point_msg = PointStamped()
        closest_point_msg.header.frame_id = "map"
        closest_point_msg.point.x = debug_points["closest_point"][0]
        closest_point_msg.point.y = debug_points["closest_point"][1]
        self.target_point_pub.publish(target_point_msg)
        self.closest_point_pub.publish(closest_point_msg)

        self.executed_trajectory.append(PoseStamped(pose=msg.pose.pose, header=msg.header))
        executed_path_msg = Path()
        executed_path_msg.header.frame_id = "odom"
        executed_path_msg.poses = self.executed_trajectory
        self.executed_path_pub.publish(executed_path_msg)

        if traj_done:
            self.get_logger().info("Reached the end of the trajectory")
            self.cmd_vel_pub.publish(Twist()) # stop the robot
            self.planned_trajectory = []
            self.initial_pose = None
            self.goal_pose = None
            return

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
