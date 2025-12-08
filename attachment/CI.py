#!/usr/bin/env python3
"""
Fuzzy Logic Controller for Robot Behavior
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import numpy as np
from enum import Enum
import time
import matplotlib.pyplot as plt

class Action(Enum):
    FOLLOW_OWNER_HIGH_VEL = "FOLLOW_OWNER_HIGH_VEL"
    FOLLOW_OWNER_MID_VEL = "FOLLOW_OWNER_MID_VEL"
    FOLLOW_STRANGER_LOW_VEL = "FOLLOW_STRANGER_LOW_VEL"
    WAIT_AT_DOOR = "WAIT_AT_DOOR"
    PICK_TARGET = "pick_target"
    DELIVER_TO_OWNER = "DELIVER_TO_OWNER"
    PLACE_TARGET = "place_target"
    IDLE = "IDLE"

class FuzzySet:
    """Simple fuzzy set"""
    def __init__(self, name, left, peak, right):
        self.name = name
        self.left = left
        self.peak = peak
        self.right = right
    
    def membership(self, x):
        if x <= self.left or x >= self.right:
            return 0
        elif x <= self.peak:
            return (x - self.left) / (self.peak - self.left)
        else:
            return (self.right - x) / (self.right - self.peak)

class FuzzyController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller')
        
        # Subscribers for object positions
        self.create_subscription(Pose2D, '/robot_pose', self.robot_callback, 10)
        self.create_subscription(Pose2D, '/toy_pose', self.toy_callback, 10)
        self.create_subscription(Pose2D, '/owner_pose', self.owner_callback, 10)
        self.create_subscription(Pose2D, '/stranger_pose', self.stranger_callback, 10)
        
        # Room distances 
        self.room_width = 360  # cm
        self.room_height = 225  # cm
        self.max_distance = np.sqrt(self.room_width**2 + self.room_height**2)  # ~424 cm
        
        # Publisher for action commands
        self.action_pub = self.create_publisher(String, '/current_action', 10)
        
        # State variables
        self.robot_pose = None
        self.toy_pose = None
        self.owner_pose = None
        self.stranger_pose = None
        self.is_holding_toy = False
        self.last_action_time = time.time()
        
        self.distance_sets = [
            # Very close: 0-50cm (0-0.5m) - good for pickup distance
            FuzzySet("very_close", 0, 0, 50),
            
            # Close: 30-130cm (0.3-1.3m) - comfortable following distance
            FuzzySet("close", 30, 80, 130),
            
            # Medium: 100-260cm (1-2.6m) - normal interaction distance
            FuzzySet("medium", 100, 180, 260),
            
            # Far: 230-400cm (2.3-4m) - across the room
            FuzzySet("far", 230, 300, 400),
            
            # Very far: 350-450cm (3.5-4.5m) - opposite corner
            FuzzySet("very_far", 350, 400, 450),
        ]        
        # Define fuzzy rules
        self.rules = self.define_rules()
        
        # Timer for decision making (10Hz)
        self.decision_timer = self.create_timer(0.1, self.decision_loop)
        
        self.get_logger().info("Fuzzy Controller initialized")
    
    def define_rules(self):
        """Define fuzzy if-then rules"""
        rules = []
        
        # Rule 1: If toy is very_close AND not holding -> pickup
        rules.append({
            'conditions': [('toy', 'very_close')],
            'action': Action.PICK_TARGET,
            'weight': 1.0
        })
        
        # Rule 2: If owner is far AND not holding -> follow fast
        rules.append({
            'conditions': [('owner', 'far'), ('owner', 'very_far')],
            'action': Action.FOLLOW_OWNER_HIGH_VEL,
            'weight': 0.9
        })
        
        # Rule 3: If owner is medium AND not holding -> follow medium
        rules.append({
            'conditions': [('owner', 'medium')],
            'action': Action.FOLLOW_OWNER_MID_VEL,
            'weight': 0.8
        })
        
        # Rule 4: If stranger is medium AND owner not visible -> follow slowly
        rules.append({
            'conditions': [('stranger', 'medium'), ('owner', 'very_far')],
            'action': Action.FOLLOW_STRANGER_LOW_VEL,
            'weight': 0.7
        })
        
        # Rule 5: If holding toy AND owner is close -> deliver
        rules.append({
            'conditions': [('owner', 'close')],
            'action': Action.DELIVER_TO_OWNER,
            'condition_check': lambda: self.is_holding_toy,
            'weight': 1.0
        })
        
        # Rule 6: If holding toy AND owner not visible -> place
        rules.append({
            'conditions': [('owner', 'very_far')],
            'action': Action.PLACE_TARGET,
            'condition_check': lambda: self.is_holding_toy,
            'weight': 0.6
        })
        
        # Rule 7: If no one visible for a while -> wait at door
        rules.append({
            'conditions': [('owner', 'very_far'), ('stranger', 'very_far')],
            'action': Action.WAIT_AT_DOOR,
            'weight': 0.5
        })
        
        return rules
    
    def robot_callback(self, msg):
        self.robot_pose = msg
        # self.get_logger().info(f"Robot at: ({msg.x:.2f}, {msg.y:.2f}), θ: {msg.theta:.2f}")
    
    def toy_callback(self, msg):
        self.toy_pose = msg
        # self.get_logger().info(f"Toy at: ({msg.x:.2f}, {msg.y:.2f})")
    
    def owner_callback(self, msg):
        self.owner_pose = msg
        # self.get_logger().info(f"Owner at: ({msg.x:.2f}, {msg.y:.2f}), θ: {msg.theta:.2f}")
    
    def stranger_callback(self, msg):
        self.stranger_pose = msg
        # self.get_logger().info(f"Stranger at: ({msg.x:.2f}, {msg.y:.2f}), θ: {msg.theta:.2f}")
    
    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        if pose1 is None or pose2 is None:
            return float('inf')
        return np.sqrt((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)
    
    def calculate_angle_difference(self, angle1, angle2):
        """Calculate smallest difference between two angles (in radians)"""
        diff = angle1 - angle2
        return np.arctan2(np.sin(diff), np.cos(diff))
    
    def fuzzify_distance(self, distance):
        """Convert crisp distance to fuzzy membership values"""
        memberships = {}
        for fs in self.distance_sets:
            memberships[fs.name] = fs.membership(min(distance, 5.0))
        return memberships
    
    def evaluate_rule(self, rule, fuzzy_values):
        """Evaluate a single fuzzy rule"""
        if 'condition_check' in rule:
            if not rule['condition_check']():
                return 0
        
        rule_strength = 0
        for obj, set_name in rule['conditions']:
            if obj in fuzzy_values and set_name in fuzzy_values[obj]:
                # Use OR logic between conditions (take max for same object)
                rule_strength = max(rule_strength, fuzzy_values[obj][set_name])
        
        return rule_strength * rule.get('weight', 1.0)
    
    def decide_action(self):
        """Main decision function using fuzzy logic"""
        if None in [self.robot_pose, self.toy_pose, self.owner_pose, self.stranger_pose]:
            self.get_logger().warn("Waiting for all position data...")
            return Action.IDLE
        
        # Calculate distances
        dist_to_toy = self.calculate_distance(self.robot_pose, self.toy_pose)
        dist_to_owner = self.calculate_distance(self.robot_pose, self.owner_pose)
        dist_to_stranger = self.calculate_distance(self.robot_pose, self.stranger_pose)
        
        # Check if we should pick up toy (hard rule - highest priority)
        if not self.is_holding_toy and dist_to_toy < 0.3:
            self.is_holding_toy = True
            self.get_logger().info("Picking up toy!")
            return Action.PICK_TARGET
        
        # Fuzzify distances
        fuzzy_toy = self.fuzzify_distance(dist_to_toy)
        fuzzy_owner = self.fuzzify_distance(dist_to_owner)
        fuzzy_stranger = self.fuzzify_distance(dist_to_stranger)
        
        fuzzy_values = {
            'toy': fuzzy_toy,
            'owner': fuzzy_owner,
            'stranger': fuzzy_stranger
        }
        
        # Evaluate all rules
        action_scores = {}
        for rule in self.rules:
            score = self.evaluate_rule(rule, fuzzy_values)
            if score > 0:
                action = rule['action']
                if action not in action_scores or score > action_scores[action]:
                    action_scores[action] = score
        
        # Choose action with highest score
        if action_scores:
            best_action = max(action_scores.items(), key=lambda x: x[1])[0]
            
            # Special handling for deliver/place actions
            if best_action == Action.DELIVER_TO_OWNER and dist_to_owner < 0.5:
                self.is_holding_toy = False
                self.get_logger().info("Delivered toy to owner!")
            elif best_action == Action.PLACE_TARGET:
                self.is_holding_toy = False
                self.get_logger().info("Placed toy down.")
            
            return best_action
        
        return Action.IDLE
    def visualize_fuzzy_sets():

        sets = [
            FuzzySet("very_close", 0, 0, 50),
            FuzzySet("close", 30, 80, 130),
            FuzzySet("medium", 100, 180, 260),
            FuzzySet("far", 230, 300, 400),
            FuzzySet("very_far", 350, 400, 450),
        ]
        
        x = np.linspace(0, 450, 500)  # 0 to 450cm
        plt.figure(figsize=(12, 5))
        
        colors = ['red', 'blue', 'green', 'orange', 'purple']
        for fs, color in zip(sets, colors):
            y = [fs.membership(xi) for xi in x]
            plt.plot(x, y, label=f'{fs.name} ({fs.left}-{fs.right}cm)', 
                    color=color, linewidth=2)
        
        # Add room boundaries
        plt.axvline(x=360, color='gray', linestyle='--', alpha=0.5, label='Room Width (360cm)')
        plt.axvline(x=225, color='gray', linestyle=':', alpha=0.5, label='Room Height (225cm)')
        
        plt.title(f"Fuzzy Sets for {360}x{225} Room (in cm)")
        plt.xlabel("Distance (cm)")
        plt.ylabel("Membership Degree")
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Add action zones
        zones = [
            (0, 30, "Pickup Zone", 'lightgreen', 0.2),
            (30, 130, "Interaction Zone", 'lightblue', 0.1),
            (130, 360, "Following Zone", 'lightyellow', 0.1),
        ]
        
        for left, right, label, color, alpha in zones:
            plt.axvspan(left, right, alpha=alpha, color=color, label=label)
        
        plt.tight_layout()
        plt.show()

    def decision_loop(self):
        """Main decision loop - called by timer"""
        if self.robot_pose is None:
            return
        
        action = self.decision_with_debug()
        
        # Publish action
        msg = String()
        msg.data = action.value
        self.action_pub.publish(msg)
        
        # Log decision occasionally
        if time.time() - self.last_action_time > 2.0:
            self.get_logger().info(f"Action: {action.value}")
            self.last_action_time = time.time()
    
    def decision_with_debug(self):
        """Decision function with debug output"""
        if None in [self.robot_pose, self.toy_pose, self.owner_pose, self.stranger_pose]:
            return Action.IDLE
        
        # Calculate and log distances
        dist_to_toy = self.calculate_distance(self.robot_pose, self.toy_pose)
        dist_to_owner = self.calculate_distance(self.robot_pose, self.owner_pose)
        dist_to_stranger = self.calculate_distance(self.robot_pose, self.stranger_pose)
        
        self.get_logger().debug(
            f"Distances - Toy: {dist_to_toy:.2f}m, "
            f"Owner: {dist_to_owner:.2f}m, "
            f"Stranger: {dist_to_stranger:.2f}m"
        )
        
        # Priority 1: Pick up toy if very close
        if not self.is_holding_toy and dist_to_toy < 0.3:
            self.is_holding_toy = True
            return Action.PICK_TARGET
        
        # Simple fuzzy rules (can be expanded)
        if not self.is_holding_toy:
            if self.owner_pose:
                if dist_to_owner > 2.0:
                    return Action.FOLLOW_OWNER_HIGH_VEL
                elif dist_to_owner > 1.0:
                    return Action.FOLLOW_OWNER_MID_VEL
            elif self.stranger_pose and dist_to_stranger > 1.5:
                return Action.FOLLOW_STRANGER_LOW_VEL
        else:
            # Holding toy
            if self.owner_pose:
                if dist_to_owner < 0.5:
                    self.is_holding_toy = False
                    return Action.DELIVER_TO_OWNER
                else:
                    return Action.FOLLOW_OWNER_HIGH_VEL
            else:
                self.is_holding_toy = False
                return Action.PLACE_TARGET
        
        return Action.WAIT_AT_DOOR

def main(args=None):
    rclpy.init(args=args)
    controller = FuzzyController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down fuzzy controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()