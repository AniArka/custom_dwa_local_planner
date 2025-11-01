#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class DWAController(Node):
    def __init__(self):
        super().__init__('dwa_controller')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_vel', 0.12),
                ('min_vel', 0.05),
                ('max_rot_vel', 0.8),
                ('max_accel', 0.3),
                ('max_rot_accel', 0.5),
                ('dt', 0.2),
                ('predict_time', 2.0),
                ('velocity_samples', 8),
                ('angular_velocity_samples', 8),
                ('goal_tolerance', 0.3),
                ('obstacle_safe_distance', 0.5),
                ('robot_radius', 0.2),
            ]
        )
        
        # Get parameters
        self.max_vel = self.get_parameter('max_vel').value
        self.min_vel = self.get_parameter('min_vel').value
        self.max_rot_vel = self.get_parameter('max_rot_vel').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_rot_accel = self.get_parameter('max_rot_accel').value
        self.dt = self.get_parameter('dt').value
        self.predict_time = self.get_parameter('predict_time').value
        self.velocity_samples = self.get_parameter('velocity_samples').value
        self.angular_velocity_samples = self.get_parameter('angular_velocity_samples').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_safe_distance = self.get_parameter('obstacle_safe_distance').value
        self.robot_radius = self.get_parameter('robot_radius').value
        
        # State variables
        self.current_pose = None
        self.current_velocity = [0.0, 0.0]
        self.laser_data = None
        self.goal_point = Point(x=2.0, y=0.0, z=0.0) 
        self.odom_received = False
        self.laser_received = False
        self.goal_reached = False
        
        # Laser configuration
        self.laser_angle_min = 0.0
        self.laser_angle_max = 6.28
        self.laser_angle_increment = 0.0175
        
        # Navigation state
        self.stuck_counter = 0
        self.last_goal_distance = float('inf')
        self.control_iteration = 0
        
        # Visualization
        self.marker_id_counter = 0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.3, self.control_loop)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("DWA CONTROLLER - COMPLETE WORKING VERSION")
        self.get_logger().info(f"Goal: ({self.goal_point.x}, {self.goal_point.y})")
        self.get_logger().info(f"Goal tolerance: {self.goal_tolerance}m")
        self.get_logger().info("=" * 60)

    def publish_goal_marker(self):
        """Publishing a marker for the goal position"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position = self.goal_point
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.goal_marker_pub.publish(marker)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_velocity[0] = msg.twist.twist.linear.x
        self.current_velocity[1] = msg.twist.twist.angular.z
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("ODOMETRY RECEIVED")

    def laser_callback(self, msg):
        self.laser_data = msg
        if not self.laser_received:
            self.laser_received = True
            self.laser_angle_min = msg.angle_min
            self.laser_angle_max = msg.angle_max
            self.laser_angle_increment = msg.angle_increment
            self.get_logger().info("LASER SCAN RECEIVED")

    def get_robot_pose(self):
        if self.current_pose is None:
            return 0.0, 0.0, 0.0
        
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        orientation = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return x, y, yaw

    def calculate_distance_to_goal(self):
        """Calculate current distance to goal"""
        if self.current_pose is None or self.goal_reached:
            return float('inf')
        
        x, y, _ = self.get_robot_pose()
        distance = math.sqrt((self.goal_point.x - x)**2 + (self.goal_point.y - y)**2)
        
        # Log progress every 10 iterations
        if self.control_iteration % 10 == 0:
            self.get_logger().info(f"Distance to goal: {distance:.3f}m (tolerance: {self.goal_tolerance}m)")
            self.get_logger().info(f"Robot: ({x:.3f}, {y:.3f}) | Goal: ({self.goal_point.x:.3f}, {self.goal_point.y:.3f})")
        
        return distance

    def is_goal_reached(self):
        """Check if goal is reached"""
        if self.goal_reached:
            return True
            
        distance = self.calculate_distance_to_goal()
        
        if distance < self.goal_tolerance:
            self.goal_reached = True
            x, y, _ = self.get_robot_pose()
            self.get_logger().info("🎉" * 20)
            self.get_logger().info("🎯 GOAL REACHED! 🎯")
            self.get_logger().info(f"Final position: ({x:.3f}, {y:.3f})")
            self.get_logger().info(f"Goal position: ({self.goal_point.x:.3f}, {self.goal_point.y:.3f})")
            self.get_logger().info(f"Final distance: {distance:.3f}m")
            self.get_logger().info("🎉" * 20)
            return True
            
        return False

    def angle_to_laser_index(self, angle):
        """Convert angle in laser frame to laser array index"""
        angle = angle % (2 * math.pi)
        if angle < 0:
            angle += 2 * math.pi
            
        if angle < self.laser_angle_min or angle > self.laser_angle_max:
            return -1
            
        index = int((angle - self.laser_angle_min) / self.laser_angle_increment)
        
        if 0 <= index < len(self.laser_data.ranges):
            return index
        return -1

    def get_min_distance_in_sector(self, center_angle, width_degrees):
        """Get minimum distance in a sector"""
        if self.laser_data is None:
            return float('inf')
            
        width_rad = math.radians(width_degrees)
        half_width = width_rad / 2
        
        start_angle = center_angle - half_width
        end_angle = center_angle + half_width
        
        start_index = self.angle_to_laser_index(start_angle)
        end_index = self.angle_to_laser_index(end_angle)
        
        if start_index == -1 or end_index == -1:
            return float('inf')
        
        min_distance = float('inf')
        
        if start_index <= end_index:
            for i in range(start_index, end_index + 1):
                if i < len(self.laser_data.ranges):
                    distance = self.laser_data.ranges[i]
                    if not math.isinf(distance) and distance > 0.05:
                        min_distance = min(min_distance, distance)
        else:
            for i in range(start_index, len(self.laser_data.ranges)):
                distance = self.laser_data.ranges[i]
                if not math.isinf(distance) and distance > 0.05:
                    min_distance = min(min_distance, distance)
            for i in range(0, end_index + 1):
                distance = self.laser_data.ranges[i]
                if not math.isinf(distance) and distance > 0.05:
                    min_distance = min(min_distance, distance)
        
        return min_distance if min_distance != float('inf') else 10.0

    def is_trajectory_safe(self, trajectory):
        """Check if trajectory is safe from obstacles"""
        if self.laser_data is None:
            return True
            
        robot_x, robot_y, robot_theta = self.get_robot_pose()
        
        for i, point in enumerate(trajectory[:5]):
            x, y, _ = point
            
            dx = x - robot_x
            dy = y - robot_y
            distance_to_point = math.sqrt(dx**2 + dy**2)
            angle_global = math.atan2(dy, dx)
            
            angle_laser = angle_global - robot_theta
            angle_laser = angle_laser % (2 * math.pi)
            if angle_laser < 0:
                angle_laser += 2 * math.pi
            
            min_distance = self.get_min_distance_in_sector(angle_laser, 20)
            
            if min_distance < distance_to_point + 0.1:
                if min_distance < self.robot_radius + 0.15:
                    return False
                        
        return True

    def calculate_goal_heading_error(self):
        """Calculate heading error to goal"""
        x, y, theta = self.get_robot_pose()
        goal_x, goal_y = self.goal_point.x, self.goal_point.y
        
        # Calculate desired heading to goal
        desired_theta = math.atan2(goal_y - y, goal_x - x)
        
        # Calculate heading error (normalized to [-pi, pi])
        heading_error = desired_theta - theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        return heading_error

    def generate_velocity_samples(self, v_current, w_current):
        """Generate velocity samples"""
        velocities = []
        
        # Get goal information
        heading_error = self.calculate_goal_heading_error()
        distance_to_goal = self.calculate_distance_to_goal()
        
        front_clearance = self.get_min_distance_in_sector(0.0, 60)
        
        # Adaptive speed based on clearance and goal proximity
        if front_clearance < 0.8:
            adaptive_max_vel = self.max_vel * (front_clearance / 0.8)
        else:
            adaptive_max_vel = self.max_vel
            
        # Slow down when close to goal
        if distance_to_goal < 1.0:
            adaptive_max_vel = min(adaptive_max_vel, self.max_vel * (distance_to_goal / 1.0))
        
        adaptive_max_vel = max(adaptive_max_vel, self.min_vel)
        
        # Generate samples
        v_samples = np.linspace(self.min_vel, adaptive_max_vel, 3)
        w_samples = np.linspace(-0.4, 0.4, 5)
        
        # Always include goal-directed motion
        goal_v = min(adaptive_max_vel, self.min_vel + 0.05)
        goal_w = np.clip(heading_error * 2.0, -0.6, 0.6)
        velocities.append((goal_v, goal_w))
        
        for v in v_samples:
            for w in w_samples:
                if abs(v - v_current) <= self.max_accel * self.dt and \
                   abs(w - w_current) <= self.max_rot_accel * self.dt:
                    velocities.append((v, w))
        
        return velocities

    def predict_trajectory(self, x, y, theta, v, w):
        """Predict trajectory for given velocity command"""
        trajectory = []
        current_x, current_y, current_theta = x, y, theta
        steps = int(self.predict_time / self.dt)
        
        for i in range(steps):
            trajectory.append((current_x, current_y, current_theta))
            current_x += v * math.cos(current_theta) * self.dt
            current_y += v * math.sin(current_theta) * self.dt
            current_theta += w * self.dt
            
        return trajectory

    def evaluate_trajectory(self, trajectory, v, w):
        """Evaluate trajectory using cost function"""
        if self.goal_point is None:
            return float('inf')
        
        # Safety check first
        if not self.is_trajectory_safe(trajectory):
            return float('inf')
        
        # Goal distance cost
        final_point = trajectory[-1]
        goal_x, goal_y = self.goal_point.x, self.goal_point.y
        traj_x, traj_y, _ = final_point
        distance_to_goal = math.sqrt((goal_x - traj_x)**2 + (goal_y - traj_y)**2)
        distance_cost = distance_to_goal * 2.0
        
        # Turn penalty (prefer straight motion)
        turn_penalty = abs(w) * 2.0
        
        # Speed bonus (prefer reasonable speed)
        speed_bonus = -v * 0.3
        
        total_cost = distance_cost + turn_penalty + speed_bonus
        return total_cost

    def visualize_trajectories(self, trajectories, best_trajectory, best_velocity):
        """Visualize trajectories in RViz - THIS WAS MISSING!"""
        marker_array = MarkerArray()
        self.marker_id_counter = 0
        
        # Clear previous markers
        delete_marker = Marker()
        delete_marker.header.frame_id = "odom"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Visualize each trajectory
        for i, (trajectory, (v, w)) in enumerate(trajectories):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectories"
            marker.id = self.marker_id_counter
            self.marker_id_counter += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            marker.scale.x = 0.03
            
            if trajectory == best_trajectory:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.scale.x = 0.05
            else:
                if abs(w) < 0.1:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 0.6
                elif w > 0:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.6
                else:
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0
                    marker.color.a = 0.6
            
            for point in trajectory:
                x, y, theta = point
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.1
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

    def control_loop(self):
        """Main control loop"""
        self.control_iteration += 1
        
        if not self.odom_received or not self.laser_received:
            if self.control_iteration % 10 == 0:
                self.get_logger().warn("Waiting for sensor data...")
            return
        
        # FIRST: Check if goal is reached
        if self.is_goal_reached():
            self.get_logger().info("STOPPING ROBOT - Goal reached!")
            self.stop_robot()
            return
        
        # Get current robot state
        x, y, theta = self.get_robot_pose()
        v_current, w_current = self.current_velocity
        
        # Calculate goal information for logging
        heading_error = self.calculate_goal_heading_error()
        
        # Log goal approach status
        if self.control_iteration % 5 == 0:
            self.get_logger().info(f"Goal heading error: {math.degrees(heading_error):.1f}°")
        
        # Stuck detection
        current_distance = self.calculate_distance_to_goal()
        if current_distance >= self.last_goal_distance:
            self.stuck_counter += 1
            if self.stuck_counter > 3:
                self.get_logger().warn(f"Stuck detected - distance increasing: {current_distance:.3f}m")
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)
        self.last_goal_distance = current_distance
        
        # If stuck for too long, use recovery behavior
        if self.stuck_counter > 6:
            self.get_logger().warn("Using recovery behavior - turning toward goal")
            # Turn toward goal
            w = np.clip(heading_error * 3.0, -0.8, 0.8)
            self.publish_velocity(0.0, w)
            self.stuck_counter = 0
            return
        
        # Use DWA for navigation
        velocities = self.generate_velocity_samples(v_current, w_current)
        
        if not velocities:
            self.get_logger().warn("No admissible velocities - turning toward goal")
            w = np.clip(heading_error * 2.0, -0.6, 0.6)
            self.publish_velocity(self.min_vel, w)
            return
        
        best_cost = float('inf')
        best_velocity = (0.0, 0.0)
        best_trajectory = None
        all_trajectories = []
        valid_trajectories = 0
        
        for v, w in velocities:
            trajectory = self.predict_trajectory(x, y, theta, v, w)
            all_trajectories.append((trajectory, (v, w)))
            
            cost = self.evaluate_trajectory(trajectory, v, w)
            
            if cost < float('inf'):
                valid_trajectories += 1
                
            if cost < best_cost:
                best_cost = cost
                best_velocity = (v, w)
                best_trajectory = trajectory
        
        # Apply the best velocity command
        if best_cost < float('inf') and valid_trajectories > 0:
            self.publish_velocity(best_velocity[0], best_velocity[1])
            
            front_clearance = self.get_min_distance_in_sector(0.0, 60)
            motion_type = "STRAIGHT" if abs(best_velocity[1]) < 0.1 else "TURNING"
            
            self.get_logger().info(f"🚗 {motion_type}: v={best_velocity[0]:.3f}, w={best_velocity[1]:.3f}, "
                                 f"dist={current_distance:.3f}m")
        else:
            self.get_logger().warn("⚠️ No safe trajectories - turning toward goal")
            w = np.clip(heading_error * 2.0, -0.6, 0.6)
            self.publish_velocity(self.min_vel, w)
        
        # Visualize trajectories (THIS WAS MISSING!)
        self.visualize_trajectories(all_trajectories, best_trajectory, best_velocity)

    def publish_velocity(self, v, w):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("🛑 ROBOT STOPPED")

    def destroy_node(self):
        """Cleanup"""
        if not self.goal_reached:
            self.get_logger().info("CONTROLLER SHUTDOWN - Goal not reached")
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    dwa_controller = DWAController()
    
    try:
        rclpy.spin(dwa_controller)
    except KeyboardInterrupt:
        dwa_controller.get_logger().info("Keyboard interrupt - shutting down")
    finally:
        dwa_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()