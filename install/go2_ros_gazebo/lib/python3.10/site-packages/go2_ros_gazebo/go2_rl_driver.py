#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import torch
import torch.nn as nn
import numpy as np
import os

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Actor(nn.Module):
    def __init__(self, num_obs, num_actions):
        super().__init__()
        # Architecture from go2_omniverse agent_cfg.py and state_dict inspection
        # Input: 235
        # Layer 0: Linear 235 -> 512
        # Layer 1: ELU
        # Layer 2: Linear 512 -> 256
        # Layer 3: ELU
        # Layer 4: Linear 256 -> 128
        # Layer 5: ELU
        # Layer 6: Linear 128 -> 12 (Output)
        self.actor = nn.Sequential(
            nn.Linear(num_obs, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, num_actions)
        )

    def forward(self, x):
        return self.actor(x)

class Go2RLDriver(Node):
    def __init__(self):
        super().__init__('go2_rl_driver')
        
        # --- Configuration ---
        self.decimation = 4
        self.loop_rate = 50.0 
        self.num_obs = 235 
        self.num_actions = 12
        self.action_scale = 0.25
        self.device = torch.device("cpu") # Use CPU for safety/compatibility on typical dev machines
        
        # Default Joint Positions (Standing)
        # Corrected Rear Thighs to 0.8 to match Front (Symmetric Standing)
        self.default_dof_pos = torch.tensor(
            [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 0.8, -1.5],
            dtype=torch.float, device=self.device
        )
        
        # Mappings
        self.joint_names_rl = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]
        
        self.joint_map = {name: i for i, name in enumerate(self.joint_names_rl)}

        # Load Model
        pkg_share = get_package_share_directory('go2_ros_gazebo')
        model_path = os.path.join(pkg_share, 'checkpoints', 'model_7850.pt')
        self.get_logger().info(f"Loading weights from: {model_path}")
        
        try:
            # 1. Initialize Network
            self.model = Actor(self.num_obs, self.num_actions).to(self.device)
            
            # 2. Load Checkpoint
            checkpoint = torch.load(model_path, map_location=self.device)
            state_dict = checkpoint['model_state_dict']
            
            # 3. Clean State Dict (Remove 'actor.' prefix if needed or match sequential)
            # The checkpoint keys are likely 'actor.0.weight', etc.
            # Our model.actor is the sequential container.
            # We can construct a new dict matching our model structure.
            
            new_state_dict = {}
            for k, v in state_dict.items():
                # Checkpoint has keys like "actor.0.weight"
                # Our self.model has member "actor". So self.model.actor has keys "0.weight".
                # Pytorch load_state_dict on self.model expects "actor.0.weight".
                if k.startswith('actor.'):
                     new_state_dict[k] = v
            
            self.model.load_state_dict(new_state_dict, strict=False)
            self.model.eval()
            self.get_logger().info("Model loaded successfully!")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise e

        # Observation Scales
        self.obs_scales = {
            'lin_vel': 2.0,
            'ang_vel': 0.25,
            'dof_pos': 1.0,
            'dof_vel': 0.05,
        }

        # Buffers
        self.obs_buf = torch.zeros((1, self.num_obs), device=self.device)
        self.last_actions = torch.zeros((1, self.num_actions), device=self.device)
        
        # Sensor States
        self.base_lin_vel = torch.zeros(3, device=self.device)
        self.base_ang_vel = torch.zeros(3, device=self.device)
        self.projected_gravity = torch.tensor([0.0, 0.0, -1.0], device=self.device)
        self.commands = torch.zeros(3, device=self.device)
        self.dof_pos = torch.zeros(12, device=self.device)
        self.dof_vel = torch.zeros(12, device=self.device)
        
        # Publishers
        self.pubs = {}
        for name in self.joint_names_rl:
            topic = f"/go2/joint/{name}"
            self.pubs[name] = self.create_publisher(Float64, topic, 10)

        # Subscribers
        self.create_subscription(Imu, '/go2/imu', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Odometry, '/model/go2/odometry', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.loop_rate, self.control_loop)
        self.get_logger().info("Go2 RL Driver Running!")

    def imu_callback(self, msg):
        self.base_ang_vel[0] = msg.angular_velocity.x
        self.base_ang_vel[1] = msg.angular_velocity.y
        self.base_ang_vel[2] = msg.angular_velocity.z
        
        q = msg.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        gx = 2 * (x*z - w*y)
        gy = 2 * (y*z + w*x)
        gz = 1 - 2 * (x*x + y*y)
        self.projected_gravity[0] = -gx
        self.projected_gravity[1] = -gy
        self.projected_gravity[2] = -gz
        
        # Debug Gravity every 50 steps (approx 1 sec)
        # self.get_logger().info(f"Gravity: {self.projected_gravity.cpu().numpy()}")

    def odom_callback(self, msg):
        # Odometry gives twist in child frame (base_link), which is what we need
        self.base_lin_vel[0] = msg.twist.twist.linear.x
        self.base_lin_vel[1] = msg.twist.twist.linear.y
        self.base_lin_vel[2] = msg.twist.twist.linear.z

    def cmd_vel_callback(self, msg):
        self.commands[0] = msg.linear.x
        self.commands[1] = msg.linear.y
        self.commands[2] = msg.angular.z

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_map:
                idx = self.joint_map[name]
                self.dof_pos[idx] = msg.position[i]
                self.dof_vel[idx] = msg.velocity[i]

    def control_loop(self):
        with torch.no_grad():
            obs_list = [
                self.base_lin_vel * self.obs_scales['lin_vel'],       
                self.base_ang_vel * self.obs_scales['ang_vel'],       
                self.projected_gravity,  
                self.commands * self.obs_scales['lin_vel'], # Commands usually scaled same as lin_vel          
                (self.dof_pos - self.default_dof_pos) * self.obs_scales['dof_pos'], 
                self.dof_vel * self.obs_scales['dof_vel'],            
                self.last_actions[0],    
                torch.zeros(187, device=self.device) 
            ]
            self.obs_buf[0] = torch.cat(obs_list)
            
            # Debug Inputs
            # self.get_logger().info(f"Obs Mean: {self.obs_buf.mean():.4f} Max: {self.obs_buf.max():.4f}")
            # self.get_logger().info(f"Grav: {self.projected_gravity.cpu().numpy()}")
            # self.get_logger().info(f"DOF Pos (scaled): { ((self.dof_pos - self.default_dof_pos) * self.obs_scales['dof_pos'])[:4] }")

            # Inference
            actions = self.model(self.obs_buf)
            
            # Process & Publish
            proc_actions = (actions * self.action_scale) + self.default_dof_pos
            action_np = proc_actions[0].cpu().numpy()

            # Sanity Check Logging
            if np.any(np.abs(action_np) > 5.0):
                 self.get_logger().warn(f"HIGH ACTION DETECTED: {action_np}")
            
            for i, name in enumerate(self.joint_names_rl):
                msg = Float64()
                msg.data = float(action_np[i])
                self.pubs[name].publish(msg)
            
            self.last_actions = actions.clone()

def main(args=None):
    rclpy.init(args=args)
    node = Go2RLDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
