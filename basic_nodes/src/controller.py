import numpy as np
import math
import rclpy
import time
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

# Contains Position, Orientation and Velocity info:- /odom topic
from nav_msgs.msg import Odometry

# Contains Angular and Linear Velocity of Robot:- /cmd_vel topic
from geometry_msgs.msg import Twist

# Lidar Sensor information:- /scan topic
from sensor_msgs.msg import LaserScan

odom_topic = '/odom'
vel_topic = '/cmd_vel'
scan_topic = '/scan'

MAX_VEL = 2.5

class Controller(Node):
    def __init__(self, x_t, y_t, k_au, k_ru, k_thetau, g_staru, eps_orientu, eps_controlu):
        super().__init__('controller_node')
        
        # Target points
        self.x_t = x_t
        self.y_t = y_t
        
        # Control Parameters for attractive and repulsive force respectively
        self.k_au = k_au
        self.k_ru = k_ru
        
        # Control parameters for orientation controller
        self.k_thetau = k_thetau

        # Parameter to reduce the influence of repulsive force
        self.g_staru = g_staru

        # Tolerance for orientation adjustment and then control
        self.eps_orientu = eps_orientu

        # Tolerence for tuning off controller
        self.eps_control = eps_controlu

        # Continously updating msgs
        self.Odometry_msg = Odometry()
        self.Laserscan_msg = LaserScan()

        self.init_time = time.time()
        # Time stamps for each msg
        self.odom_time = time.time() # Position and orientation
        self.laser_time = time.time() # Lidar Scan

        # To receive the linear and angular velocity
        self.control_vel = Twist()

        self.control_vel.linear.x = 0.0
        self.control_vel.linear.y = 0.0
        self.control_vel.linear.z = 0.0

        self.control_vel.angular.x = 0.0
        self.control_vel.angular.y = 0.0
        self.control_vel.angular.z = 0.0

        # Publisher
        self.control_pub_ = self.create_publisher(Twist, vel_topic, 10)

        # Subscriber
        self.odom_subs_ = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.laser_subs_ = self.create_subscription(LaserScan, scan_topic, self.laser_callback, 10)

        # Control frequency
        self.period = 0.05

        self.timer = self.create_timer(self.period, self.Control_function)


    def odom_callback(self, received_msg):
        self.Odometry_msg = received_msg
        self.odom_time = time.time()
    

    def laser_callback(self, received_msg):
        self.Laserscan_msg = received_msg
        self.laser_time = time.time()


    # To calculate the orientation error handling angle wrapping
    def orientation_error(self, theta, theta_d):
        # Handle angle wrapping when theta in IIIrd quadrant and theta_d in IInd quadrant
        if theta > -np.pi and theta <= -np.pi/2:
            if theta_d > np.pi/2 and theta_d <= np.pi:
                theta = theta + 2 * np.pi

        # Handle angle wrapping when theta_d in IIIrd quadrant and theta in IInd quadrant
        if theta_d > -np.pi and theta_d <= -np.pi/2:
            if theta > np.pi/2 and theta <= np.pi:
                theta_d = theta_d + 2 * np.pi

        # Calculate orientation error
        error_orientation = theta_d - theta
        return error_orientation


    # Calculate and send control actions
    def Control_function(self):
        # Control parameters for att and rep force
        k_au = self.k_au
        k_ru = self.k_ru

        # For orientation control and rep range
        k_thetau = self.k_thetau
        g_staru = self.g_staru

        # Target points
        x_t = self.x_t
        y_t = self.y_t

        # Current position
        x_curr = self.Odometry_msg.pose.pose.position.x
        y_curr = self.Odometry_msg.pose.pose.position.y

        quat = self.Odometry_msg.pose.pose.orientation
        quat_list = [i for i in quat]
        # Current orientation (yaw)
        roll, pitch, yaw = euler_from_quaternion(quaternion=quat_list)
        theta = yaw

        # Lidar measurements from sensor
        Lidar_ranges = np.array(self.Laserscan_msg.ranges)
        # Array format -> [inf, inf, ..., 2.31, 2.35, 2.4, ..., inf] throughout the defined range

        # Lidar angle parameters
        angle_min = self.Laserscan_msg.angle_min
        angle_inc = self.Laserscan_msg.angle_increment

        # Calculate Attractive Force towards goal:- F_att = -k_a * (x_current - x_goal)
        AF = - k_au * np.array([[x_curr - x_t], [y_curr - y_t]])

        # Find indices where obstacles are detected (non-infinite values)
        finite_indices = np.where(~np.isinf(Lidar_ranges))[0]
        obstacle_present = len(finite_indices) > 0

        if obstacle_present:
            # Find gaps between consecutive indices to separate different obstacles
            diff_arr = np.diff(finite_indices)
            obs_indices = np.where(np.abs(diff_arr) > 1)[0] + 1
            
            # Split indices into subarrays where each subarray corresponds to one obstacle
            partition_arr = np.split(finite_indices, obs_indices)

            # Calculate global angles of all lidar rays in fixed frame (gamma)
            angles = angle_min + finite_indices * angle_inc + theta
            distances = Lidar_ranges[finite_indices]

            # Convert polar coordinates to Cartesian coordinates in global frame
            x_o = x_curr * np.ones(distances.shape) + distances * np.cos(angles)
            y_o = y_curr * np.ones(distances.shape) + distances * np.sin(angles)

            # Find minimum distance and angle for each obstacle
            min_distances = []
            min_distances_angles = []

            for i in range(len(partition_arr)):
                tmp_arr = Lidar_ranges[partition_arr[i]]
                min_idx = np.argmin(tmp_arr)
                min_distances.append(min(tmp_arr))
                # Store angle of closest point on obstacle in robot frame
                min_distances_angles.append(angle_min + angle_inc * partition_arr[i][min_idx])

            # Convert closest obstacle points to Cartesian coordinates in global frame
            x_o_min = []
            y_o_min = []

            for i in range(len(min_distances)):
                # Transform from robot frame to global frame
                x_o_min.append(x_curr + min_distances[i] * np.cos(min_distances_angles[i] + theta))
                y_o_min.append(y_curr + min_distances[i] * np.sin(min_distances_angles[i] + theta))

            # Calculate repulsive force gradient for each obstacle
            g_vals = []
            grad_vals = []

            for i in range(len(min_distances)):
                # Distance from robot to closest point on obstacle
                g_val = np.sqrt((x_curr - x_o_min[i]) ** 2 + (y_curr - y_o_min[i]) ** 2)
                g_vals.append(g_val)

                # Calculate repulsive potential gradient only if obstacle is within influence range
                if g_val <= g_staru:
                    # Repulsive potential gradient magnitude:- grad(U_rep) = k_r * (1/g* - 1/g) * (1/g^3)
                    pr = k_ru * ((1 / g_staru) - (1 / g_vals[i])) * (1 / (g_vals[i]) ** 3)
                    # Direction from obstacle to robot (repulsive direction)
                    grad_vals_i = pr * np.array([[x_curr - x_o_min[i]], [y_curr - y_o_min[i]]])
                    grad_vals.append(grad_vals_i)
                else:
                    # No repulsive force if obstacle is beyond influence range
                    grad_vals.append(np.array([[0], [0]]))

            # Sum all repulsive force gradients
            RF = np.array([[0], [0]])

            for i in range(len(grad_vals)):
                RF = RF + grad_vals[i]

            # Repulsive Force is negative gradient of potential
            RF = -RF

        # Calculate total force (Attractive + Repulsive)
        if obstacle_present:
            F = AF + RF
        else:
            F = AF
        
        # Calculate desired orientation from force vector
        theta_d = math.atan2(F[1, 0], F[0, 0])
        e_orrient = self.orientation_error(theta, theta_d)

        # Stop robot if force magnitude is below control tolerance (goal reached)
        if np.linalg.norm(F, 2) < self.eps_control:
            theta_vel = 0.0
            x_vel = 0.0
        else:
            # If orientation error is large, only rotate (no forward motion)
            if np.abs(e_orrient) > self.eps_orientu:
                theta_vel = k_thetau * e_orrient
                x_vel = 0.0
            # If oriented correctly, move forward and fine-tune orientation
            else:
                theta_vel = k_thetau * e_orrient
                x_vel = np.linalg.norm(F, 2)
            # Limit maximum linear velocity for safety
            if (np.abs(x_vel) > MAX_VEL):
                x_vel = MAX_VEL

        # Set linear velocity (forward/backward)
        self.control_vel.linear.x = x_vel
        self.control_vel.linear.y = 0.0  # No lateral motion for differential drive
        self.control_vel.linear.z = 0.0  # No vertical motion
        
        # Set angular velocity (rotation about z-axis)
        self.control_vel.angular.x = 0.0
        self.control_vel.angular.y = 0.0
        self.control_vel.angular.z = theta_vel

        # Publish control commands to robot
        print("Sending control velocity!")
        self.control_pub_.publish(self.control_vel)
            

def main(args=None):
    rclpy.init(args=args)
    
    # Control parameters for potential field navigation
    x_t = 5.0  # Target x position in meters
    y_t = 5.0  # Target y position in meters
    k_au = 1.0  # Attractive force gain (higher = faster approach to goal)
    k_ru = 1.0  # Repulsive force gain (higher = stronger obstacle avoidance)
    k_thetau = 2.0  # Orientation controller gain (higher = faster rotation)
    g_staru = 1.0  # Repulsive force influence range in meters
    eps_orientu = 0.1  # Orientation tolerance in radians before moving forward
    eps_controlu = 0.1  # Control tolerance - stop when force magnitude below this
    
    # Create controller node with parameters
    node = Controller(x_t, y_t, k_au, k_ru, k_thetau, g_staru, eps_orientu, eps_controlu)
    
    # Keep node running until interrupted
    rclpy.spin(node=node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()