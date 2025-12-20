import rclpy
from rclpy.node import Node
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import threading
import time
from typing import Optional, Tuple, Dict, Any

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState, GetEntityState


class MazeEnv(gym.Env):
    """ROS2 Gazebo Maze Environment for RL Training"""
    
    metadata = {'render_modes': ['human'], 'render_fps': 10}
    
    def __init__(self, render_mode: Optional[str] = None):
        super().__init__()
        
        # Initialize ROS2
        rclpy.init()
        self.node = Node("maze_rl_env")
        self.logger = self.node.get_logger()
        
        # Thread safety
        self.scan_lock = threading.Lock()
        self.odom_lock = threading.Lock()
        self.scan_event = threading.Event()
        
        # State variables with thread safety
        self._scan: Optional[np.ndarray] = None
        self._position: np.ndarray = np.zeros(3)  # x, y, yaw
        self._linear_velocity: float = 0.0
        self._angular_velocity: float = 0.0
        
        # Episode tracking
        self.step_count: int = 0
        self.max_steps: int = 1000
        self.last_position: np.ndarray = np.zeros(2)
        self.start_position: np.ndarray = np.zeros(2)
        self.goal_position: np.ndarray = np.array([5.0, 5.0])  # Example goal
        
        # Subscriptions
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher
        self.cmd_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Service clients for reset
        self.reset_world_client = self.node.create_client(Empty, '/reset_world')
        self.reset_simulation_client = self.node.create_client(Empty, '/reset_simulation')
        
        # Gazebo service clients for more precise control
        self.set_entity_client = None
        self.get_entity_client = None
        
        try:
            self.set_entity_client = self.node.create_client(
                SetEntityState, '/gazebo/set_entity_state'
            )
            self.get_entity_client = self.node.create_client(
                GetEntityState, '/gazebo/get_entity_state'
            )
        except:
            self.logger.warn("Gazebo services not available, using simple reset")
        
        # Action space: [linear_velocity, angular_velocity]
        self.action_space = spaces.Box(
            low=np.array([-0.5, -1.5]),
            high=np.array([0.5, 1.5]),
            dtype=np.float32
        )
        
        # Observation space: 24 evenly sampled lidar beams + position relative to goal
        self.observation_space = spaces.Box(
            low=np.array([0.0] * 24 + [-10.0, -10.0, -np.pi]),
            high=np.array([3.5] * 24 + [10.0, 10.0, np.pi]),
            dtype=np.float32
        )
        
        # Wait for initial data
        self._wait_for_initial_data()
        
        self.logger.info("Maze Environment initialized")
    
    def _wait_for_initial_data(self, timeout: float = 5.0) -> None:
        """Wait for initial sensor data"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._scan is not None:
                self.scan_event.set()
                break
        if self._scan is None:
            self.logger.error("Failed to receive initial scan data")
    
    def scan_callback(self, msg: LaserScan) -> None:
        """Process laser scan data with thread safety"""
        ranges = np.array(msg.ranges)
        
        # Replace inf values with max range
        ranges[np.isinf(ranges)] = 3.5
        ranges[np.isnan(ranges)] = 3.5
        
        # Evenly sample 24 beams
        num_beams = len(ranges)
        indices = np.linspace(0, num_beams - 1, 24, dtype=int)
        sampled_ranges = ranges[indices]
        
        with self.scan_lock:
            self._scan = sampled_ranges.astype(np.float32)
        self.scan_event.set()
    
    def odom_callback(self, msg: Odometry) -> None:
        """Process odometry data with thread safety"""
        with self.odom_lock:
            self._position[0] = msg.pose.pose.position.x
            self._position[1] = msg.pose.pose.position.y
            
            # Extract yaw from quaternion
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._position[2] = np.arctan2(siny_cosp, cosy_cosp)
            
            self._linear_velocity = msg.twist.twist.linear.x
            self._angular_velocity = msg.twist.twist.angular.z
    
    def get_observation(self) -> np.ndarray:
        """Safely get current observation"""
        # Wait for fresh scan (with timeout)
        if not self.scan_event.wait(timeout=0.1):
            self.logger.warn("Scan data timeout")
        
        with self.scan_lock:
            scan_data = self._scan.copy() if self._scan is not None else np.ones(24)
        
        with self.odom_lock:
            position_data = self._position.copy()
        
        # Calculate position relative to goal
        dx = self.goal_position[0] - position_data[0]
        dy = self.goal_position[1] - position_data[1]
        angle_to_goal = np.arctan2(dy, dx) - position_data[2]
        
        # Normalize angle to [-pi, pi]
        angle_to_goal = np.arctan2(np.sin(angle_to_goal), np.cos(angle_to_goal))
        
        # Combine scan + relative position info
        observation = np.concatenate([
            scan_data,
            np.array([dx, dy, angle_to_goal])
        ])
        
        return observation.astype(np.float32)
    
    def get_state(self) -> Dict[str, Any]:
        """Get complete state information"""
        with self.odom_lock:
            position = self._position.copy()
            linear_vel = self._linear_velocity
            angular_vel = self._angular_velocity
        
        with self.scan_lock:
            scan = self._scan.copy() if self._scan is not None else np.ones(24)
        
        return {
            'position': position,
            'linear_velocity': linear_vel,
            'angular_velocity': angular_vel,
            'scan': scan,
            'step': self.step_count
        }
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Execute one time step
        
        Returns:
            observation, reward, terminated, truncated, info
        """
        self.step_count += 1
        
        # Publish action
        cmd = Twist()
        cmd.linear.x = float(np.clip(action[0], -0.5, 0.5))
        cmd.angular.z = float(np.clip(action[1], -1.5, 1.5))
        self.cmd_pub.publish(cmd)
        
        # Small non-blocking spin to process callbacks
        rclpy.spin_once(self.node, timeout_sec=0.0)
        
        # Get observation
        observation = self.get_observation()
        state = self.get_state()
        
        # Calculate rewards
        reward = 0.0
        
        # 1. Progress reward (distance towards goal)
        current_pos = state['position'][:2]
        prev_distance = np.linalg.norm(self.last_position - self.goal_position)
        current_distance = np.linalg.norm(current_pos - self.goal_position)
        progress = prev_distance - current_distance
        reward += progress * 2.0  # Scale progress reward
        
        # 2. Velocity reward (encourage movement)
        reward += abs(state['linear_velocity']) * 0.1
        
        # 3. Efficiency penalty (discourage unnecessary turning)
        reward -= abs(state['angular_velocity']) * 0.05
        
        # 4. Time penalty (encourage efficiency)
        reward -= 0.01
        
        # Check termination conditions
        terminated = False
        truncated = False
        
        # Collision check
        min_scan_distance = np.min(observation[:24])
        if min_scan_distance < 0.15:
            reward -= 5.0
            terminated = True
            self.logger.info(f"Episode terminated: Collision at step {self.step_count}")
        
        # Goal reached
        elif current_distance < 0.5:  # Within 0.5m of goal
            reward += 10.0
            terminated = True
            self.logger.info(f"Episode terminated: Goal reached at step {self.step_count}")
        
        # Timeout (truncated)
        elif self.step_count >= self.max_steps:
            truncated = True
            self.logger.info(f"Episode truncated: Max steps reached")
        
        # Too close to obstacle warning
        elif min_scan_distance < 0.3:
            reward -= 0.1
        
        # Update last position
        self.last_position = current_pos.copy()
        
        # Info dictionary
        info = {
            'position': current_pos.tolist(),
            'distance_to_goal': float(current_distance),
            'min_scan_distance': float(min_scan_distance),
            'step': self.step_count,
            'progress': float(progress)
        }
        
        return observation, reward, terminated, truncated, info
    
    def reset(self, *, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """
        Reset the environment
        """
        super().reset(seed=seed)
        
        # Reset episode tracking
        self.step_count = 0
        self.scan_event.clear()
        
        # Stop the robot
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
        # Reset simulation using available services
        reset_success = False
        
        # Try Gazebo reset services first
        if self.set_entity_client and self.get_entity_client:
            try:
                # Get current entity state
                get_req = GetEntityState.Request()
                get_req.name = "turtlebot3_burger"  # Adjust for your robot
                future = self.get_entity_client.call_async(get_req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                
                if future.result() is not None:
                    # Reset to start position (0, 0)
                    set_req = SetEntityState.Request()
                    set_req.state.name = "turtlebot3_burger"
                    set_req.state.pose.position.x = 0.0
                    set_req.state.pose.position.y = 0.0
                    set_req.state.pose.position.z = 0.0
                    set_req.state.pose.orientation.x = 0.0
                    set_req.state.pose.orientation.y = 0.0
                    set_req.state.pose.orientation.z = 0.0
                    set_req.state.pose.orientation.w = 1.0
                    
                    future = self.set_entity_client.call_async(set_req)
                    rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                    reset_success = True
            except Exception as e:
                self.logger.warn(f"Gazebo reset failed: {e}")
        
        # Try general reset services
        if not reset_success:
            for service_name, client in [('/reset_world', self.reset_world_client),
                                         ('/reset_simulation', self.reset_simulation_client)]:
                if client and client.service_is_ready():
                    try:
                        req = Empty.Request()
                        future = client.call_async(req)
                        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                        reset_success = True
                        self.logger.info(f"Reset using {service_name}")
                        break
                    except Exception as e:
                        self.logger.warn(f"Reset service {service_name} failed: {e}")
        
        if not reset_success:
            self.logger.warn("No reset service available - using manual reset")
            # Manual reset: stop and wait
            time.sleep(1.0)
        
        # Wait for fresh sensor data
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._scan is not None:
                break
        
        if self._scan is None:
            self.logger.error("No scan data after reset")
        
        # Get initial observation
        observation = self.get_observation()
        
        # Store start position
        with self.odom_lock:
            self.start_position = self._position[:2].copy()
            self.last_position = self._position[:2].copy()
        
        # Info dictionary
        info = {
            'reset_success': reset_success,
            'start_position': self.start_position.tolist(),
            'goal_position': self.goal_position.tolist()
        }
        
        self.logger.info(f"Environment reset. Start: {self.start_position}, Goal: {self.goal_position}")
        
        return observation, info
    
    def render(self):
        """Optional rendering method"""
        # Could publish visualization markers or log state
        state = self.get_state()
        self.logger.info(
            f"Step: {self.step_count}, "
            f"Pos: [{state['position'][0]:.2f}, {state['position'][1]:.2f}], "
            f"Min dist: {np.min(state['scan']):.2f}"
        )
    
    def close(self):
        """Cleanup"""
        # Stop robot
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
        # Small delay to ensure stop command is sent
        time.sleep(0.1)
        
        self.logger.info("Environment closed")
    
    def set_goal(self, goal_position: np.ndarray):
        """Dynamically set goal position"""
        self.goal_position = goal_position.copy()
        self.logger.info(f"New goal set: {self.goal_position}")
    
    def _spin_thread(self):
        """Optional: Separate thread for spinning"""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.001)


# Example usage and training wrapper
class FastTrainingWrapper(gym.Wrapper):
    """Wrapper for faster training without ROS2 overhead during steps"""
    
    def __init__(self, env):
        super().__init__(env)
        self.last_action = None
    
    def step(self, action):
        # Store action for potential use in reset
        self.last_action = action
        return self.env.step(action)
    
    def reset(self, **kwargs):
        # Stop any previous motion
        if self.last_action is not None:
            self.env.cmd_pub.publish(Twist())
        return self.env.reset(**kwargs)


if __name__ == "__main__":
    # Test the environment
    env = MazeEnv()
    
    try:
        # Test reset
        obs, info = env.reset()
        print(f"Initial observation shape: {obs.shape}")
        print(f"Reset info: {info}")
        
        # Test a few steps
        for i in range(10):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            print(f"Step {i}: Reward={reward:.3f}, Terminated={terminated}")
            
            if terminated or truncated:
                break
        
        env.close()
        
    except KeyboardInterrupt:
        env.close()
    finally:
        rclpy.shutdown()