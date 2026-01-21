"""
IK Solver Module for Robotic Arm Control
Uses IKPy library for inverse kinematics computation
"""

import json
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import os


class RobotIKSolver:
    """
    Inverse kinematics solver for serial robotic arms
    Supports DH parameter-based robot definitions
    """

    def __init__(self, config_path):
        """
        Initialize IK solver with robot configuration

        Args:
            config_path: Path to robot configuration JSON file
        """
        self.config_path = config_path
        self.config = self.load_config()
        self.chain = self.build_kinematic_chain()
        self.num_joints = self.config['robot_info']['num_joints']

        # Extract workspace limits
        workspace = self.config.get('workspace', {}).get('limits', {})
        self.workspace_limits = {
            'x': (workspace.get('x_min', -1.0), workspace.get('x_max', 1.0)),
            'y': (workspace.get('y_min', -1.0), workspace.get('y_max', 1.0)),
            'z': (workspace.get('z_min', 0.0), workspace.get('z_max', 1.0))
        }

        # Extract joint limits
        self.joint_limits = []
        for joint in self.config['joints']:
            limits = joint.get('limits', {})
            self.joint_limits.append({
                'min': limits.get('position_min', -3.14159),
                'max': limits.get('position_max', 3.14159)
            })

    def load_config(self):
        """Load robot configuration from JSON file"""
        with open(self.config_path, 'r') as f:
            return json.load(f)

    def build_kinematic_chain(self):
        """
        Build IKPy kinematic chain from DH parameters

        Returns:
            ikpy.chain.Chain object
        """
        links = [OriginLink()]  # Start with origin link

        for joint in self.config['joints']:
            dh = joint['dh_params']
            limits = joint['limits']

            # Create URDF-style link from DH parameters
            # IKPy uses modified DH convention
            link = URDFLink(
                name=joint['name'],
                origin_translation=[dh['a'], 0, dh['d']],
                origin_orientation=[0, 0, dh['alpha']],
                rotation=[0, 0, 1],  # Rotation around Z axis (theta)
                bounds=(limits['position_min'], limits['position_max'])
            )
            links.append(link)

        # Create chain
        chain = Chain(
            name=self.config['robot_info']['name'],
            links=links,
            active_links_mask=[False] + [True] * self.num_joints  # Origin link is passive
        )

        return chain

    def compute_ik(self, target_position, initial_guess=None, orientation_mode=None):
        """
        Compute inverse kinematics for target position

        Args:
            target_position: Target position [x, y, z] in meters
            initial_guess: Initial joint angles for IK solver (optional)
            orientation_mode: Target orientation (optional, None for position-only)

        Returns:
            tuple: (joint_angles, success, error)
                - joint_angles: numpy array of joint angles
                - success: True if solution found and valid
                - error: Position error in meters
        """
        # Convert target to numpy array
        target = np.array(target_position)

        # Check workspace limits first
        if not self.check_workspace_limits(target):
            return None, False, float('inf')

        # Prepare initial guess (use current position or home position)
        if initial_guess is None:
            initial_guess = np.array([joint['home_position'] for joint in self.config['joints']])

        # Add origin link position (0) at the beginning
        initial_position = np.concatenate([[0], initial_guess])

        # Compute IK (position-only)
        try:
            ik_solution = self.chain.inverse_kinematics(
                target_position=target,
                initial_position=initial_position,
                orientation_mode=orientation_mode
            )

            # Extract joint angles (skip origin link)
            joint_angles = ik_solution[1:]

            # Validate solution
            valid, violating_joint = self.check_joint_limits(joint_angles)
            if not valid:
                return joint_angles, False, float('inf')

            # Compute forward kinematics to verify solution
            fk_result = self.compute_fk(joint_angles)
            error = np.linalg.norm(fk_result[:3] - target)

            # Solution is successful if error is small (< 1cm)
            success = error < 0.01

            return joint_angles, success, error

        except Exception as e:
            print(f"IK computation failed: {e}")
            return None, False, float('inf')

    def compute_fk(self, joint_angles):
        """
        Compute forward kinematics

        Args:
            joint_angles: Array of joint angles

        Returns:
            numpy array: End effector position [x, y, z, ...]
        """
        # Add origin link position (0) at the beginning
        full_angles = np.concatenate([[0], joint_angles])

        # Compute FK
        transformation_matrix = self.chain.forward_kinematics(full_angles)

        # Extract position from transformation matrix
        position = transformation_matrix[:3, 3]

        return position

    def check_workspace_limits(self, position):
        """
        Check if position is within workspace limits

        Args:
            position: Target position [x, y, z]

        Returns:
            bool: True if within limits
        """
        x, y, z = position

        in_x = self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1]
        in_y = self.workspace_limits['y'][0] <= y <= self.workspace_limits['y'][1]
        in_z = self.workspace_limits['z'][0] <= z <= self.workspace_limits['z'][1]

        return in_x and in_y and in_z

    def check_joint_limits(self, joint_angles):
        """
        Check if joint angles are within limits

        Args:
            joint_angles: Array of joint angles

        Returns:
            tuple: (valid, violating_joint_index)
        """
        for i, angle in enumerate(joint_angles):
            if i >= len(self.joint_limits):
                continue

            limits = self.joint_limits[i]
            if not (limits['min'] <= angle <= limits['max']):
                return False, i

        return True, None

    def get_workspace_bounds(self):
        """Get workspace boundaries for visualization"""
        return self.workspace_limits

    def get_joint_limits(self):
        """Get joint limits"""
        return self.joint_limits

    def get_link_lengths(self):
        """Get link lengths (a parameters from DH)"""
        return [joint['dh_params']['a'] for joint in self.config['joints']]

    def update_link_lengths(self, link_lengths, base_height=0.0):
        """
        Update DH parameters with new link lengths and rebuild kinematic chain

        Args:
            link_lengths: List of link lengths (a parameters) in meters
            base_height: Base height offset (d parameter of first joint)
        """
        # Update link lengths in config
        for i, length in enumerate(link_lengths):
            if i < len(self.config['joints']):
                self.config['joints'][i]['dh_params']['a'] = length

        # Update base height
        if self.config['joints'] and base_height is not None:
            self.config['joints'][0]['dh_params']['d'] = base_height

        # Rebuild kinematic chain with new parameters
        self.chain = self.build_kinematic_chain()
        self.num_joints = len([j for j in self.config['joints']])

        print(f"[IK Solver] Updated chain with lengths: {link_lengths}, base: {base_height}")

        # Update workspace limits based on new reach
        total_reach = sum(link_lengths)
        self.workspace_limits['x'] = (-total_reach, total_reach)
        self.workspace_limits['y'] = (-total_reach, total_reach)
