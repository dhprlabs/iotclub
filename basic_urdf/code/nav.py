import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

def quaternion_to_yaw(q):
    '''
    Purpose:
    ---
    Converts a quaternion orientation to a yaw angle (in radians).

    Input Arguments:
    ---
    `q` :  [ geometry_msgs.msg.Quaternion ]
        Quaternion representing orientation.

    Returns:
    ---
    `yaw` :  [ float ]
        Yaw angle in radians.

    Example call:
    ---
    yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    '''
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PotentialFieldNavigator(Node):
    def __init__(self):
        '''
        Purpose:
        ---
        Initializes the PotentialFieldNavigator node, subscriptions, publishers, parameters, and state.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        node = PotentialFieldNavigator()
        '''
        super().__init__('nav')

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # self.lidar_sub: ROS2 subscription to LaserScan messages for obstacle detection

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.odom_sub: ROS2 subscription to Odometry messages for robot pose

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.cmd_pub: ROS2 publisher for velocity commands

        self.waypoints = [[8.0, -7.0, 0.0]]
        # self.waypoints: List of waypoints [x, y, yaw] for the robot to follow

        self.current_wp = 0
        # self.current_wp: Index of the current waypoint being targeted

        self.pos_tol = 0.3
        # self.pos_tol: Position tolerance to consider waypoint reached

        self.yaw_tol = math.radians(10)
        # self.yaw_tol: Yaw (orientation) tolerance in radians

        self.k_att = 3.7
        # self.k_att: Gain for attractive force towards the goal

        self.k_rep = 2.8
        # self.k_rep: Gain for repulsive force from obstacles

        self.rep_range = 2.2
        # self.rep_range: Distance within which obstacles exert repulsive force

        self.damping = 0.6
        # self.damping: Damping factor for velocity control

        self.x = 0.0
        # self.x: Current x-coordinate of the robot

        self.y = 0.0
        # self.y: Current y-coordinate of the robot

        self.yaw = 0.0
        # self.yaw: Current orientation (yaw) of the robot

        self.lidar_ranges = []
        # self.lidar_ranges: Latest lidar range readings

        self.timer = self.create_timer(0.1, self.navigate)
        # self.timer: ROS2 timer to periodically call the navigate function

        self.get_logger().info("[STARTED]: nav is active")

    def lidar_callback(self, msg: LaserScan):
        '''
        Purpose:
        ---
        Callback to update lidar range data from LaserScan messages.

        Input Arguments:
        ---
        `msg` : [ sensor_msgs.msg.LaserScan ]
            Incoming LaserScan message.

        Returns:
        ---
        None

        Example call:
        ---
        lidar_callback(msg)
        '''
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_max = msg.angle_max
        self.lidar_angle_increment = msg.angle_increment

    def odom_callback(self, msg: Odometry):
        '''
        Purpose:
        ---
        Callback to update robot's position and orientation from Odometry messages.

        Input Arguments:
        ---
        `msg` : [ nav_msgs.msg.Odometry ]
            Incoming Odometry message.

        Returns:
        ---
        None

        Example call:
        ---
        odom_callback(msg)
        '''
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def pos_error(self, tx, ty):
        '''
        Purpose:
        ---
        Computes the Euclidean distance between current position and target position.

        Input Arguments:
        ---
        `tx` : [ float ]
            Target x-coordinate.
        `ty` : [ float ]
            Target y-coordinate.

        Returns:
        ---
        `distance` : [ float ]
            Euclidean distance to target.

        Example call:
        ---
        error = self.pos_error(tx, ty)
        '''
        return math.sqrt((self.x - tx) ** 2 + (self.y - ty) ** 2)

    def yaw_error(self, target_yaw):
        '''
        Purpose:
        ---
        Computes the shortest angular difference between current and target yaw.

        Input Arguments:
        ---
        `target_yaw` : [ float ]
            Target yaw angle in radians.

        Returns:
        ---
        `yaw_diff` : [ float ]
            Angular difference in radians.

        Example call:
        ---
        error = self.yaw_error(target_yaw)
        '''
        return math.atan2(math.sin(target_yaw - self.yaw),
                          math.cos(target_yaw - self.yaw))
    
    def compute_force(self, goal_x, goal_y):
        '''
        Purpose:
        ---
        Calculates the combined attractive and repulsive force vectors for navigation.

        Input Arguments:
        ---
        `goal_x` : [ float ]
            Target x-coordinate.
        `goal_y` : [ float ]
            Target y-coordinate.

        Returns:
        ---
        `force` : [ numpy.ndarray ]
            Resultant force vector [fx, fy].

        Example call:
        ---
        force = self.compute_force(goal_x, goal_y)
        '''
        # Calculate attractive force towards the goal
        dx = goal_x - self.x
        dy = goal_y - self.y
        att_force = np.array([self.k_att * dx, self.k_att * dy])
        rep_force = np.array([0.0, 0.0])

        # Calculate repulsive force from obstacles if lidar data is available
        if len(self.lidar_ranges) > 0:
            # Generate angles using actual lidar scan parameters
            # angle_min: starting angle of the scan (e.g., -π for rear, 0 for forward)
            # angle_max: ending angle of the scan (e.g., π for rear, 2π for forward)
            # len(self.lidar_ranges): number of measurements in the scan
            angles = np.linspace(self.lidar_angle_min, self.lidar_angle_max, len(self.lidar_ranges))
            for r, ang in zip(self.lidar_ranges, angles):
                if r < self.rep_range:
                    # Compute obstacle position in global frame
                    # Here we have calculated dist which is same as r from lidar, It's here just to respect the procedure of Potential Field Method.
                    ox = self.x + r * math.cos(self.yaw + ang)
                    oy = self.y + r * math.sin(self.yaw + ang)
                    dist = math.sqrt((self.x - ox) ** 2 + (self.y - oy) ** 2)
                    if dist > 0.001:
                        # Repulsive force magnitude (potential field formula)
                        rep = self.k_rep * (1.0 / dist - 1.0 / self.rep_range) / (dist ** 2)
                        dir_vec = np.array([(self.x - ox) / dist, (self.y - oy) / dist])
                        rep_force += rep * dir_vec
                        # Add a small tangential component to help avoid local minima
                        tangent = np.array([-dir_vec[1], dir_vec[0]])
                        rep_force += 0.1 * tangent  

        # Return the sum of attractive and repulsive forces
        return att_force + rep_force

    def reached_waypoint(self):
        '''
        Purpose:
        ---
        Checks if the robot has reached the current waypoint.

        Input Arguments:
        ---
        None

        Returns:
        ---
        `reached` : [ bool ]
            True if waypoint is reached, else False.

        Example call:
        ---
        if self.reached_waypoint():
            ...
        '''
        if self.current_wp >= len(self.waypoints):
            return True
        
        tx, ty, tyaw = self.waypoints[self.current_wp]
        pe = self.pos_error(tx, ty)
        ye = abs(self.yaw_error(tyaw))

        return pe < self.pos_tol and ye < self.yaw_tol

    def navigate(self):
        '''
        Purpose:
        ---
        Main navigation loop: computes forces, generates velocity commands, and manages waypoints.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.navigate()
        '''
        # Stop the robot if all waypoints are reached
        if self.current_wp >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            self.get_logger().info("[FINISHED]: All waypoints reached")
            return

        tx, ty, tyaw = self.waypoints[self.current_wp]
        pe = self.pos_error(tx, ty)

        # Log current position periodically for debugging
        self.get_logger().info(f"[NAV]: Target=({tx}, {ty}), Current=({self.x:.2f}, {self.y:.2f}), Error={pe:.2f}")

        # If close enough to the waypoint, move to the next
        if pe < self.pos_tol:
            self.get_logger().info(f"[REACHED]: Waypoint {self.current_wp + 1} at position ({self.x:.2f}, {self.y:.2f})")
            self.current_wp += 1
            return

        # Compute navigation force and desired heading
        force = self.compute_force(tx, ty)
        fx, fy = force[0], force[1]
        desired_yaw = math.atan2(fy, fx)

        # Calculate yaw error and set velocity commands
        yaw_err = self.yaw_error(desired_yaw)
        cmd = Twist()
        # Set linear velocity proportional to force magnitude, within limits
        cmd.linear.x = max(0.1, min(0.3, self.damping * np.linalg.norm(force)))
        # Set angular velocity proportional to yaw error
        cmd.angular.z = 0.5 * yaw_err

        # Publish velocity command to the robot
        self.cmd_pub.publish(cmd)

def main():
    '''
    Purpose:
    ---
    Initializes ROS2, starts the PotentialFieldNavigator node, and spins.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    main()
    '''
    rclpy.init()
    node = PotentialFieldNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
