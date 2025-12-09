#!/usr/bin/env python3
"""
Fuzzy Logic Controller + Lightweight RL Layer
For robot behavior in meters
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import numpy as np
from enum import Enum
import time

# -------------------------------------------------------------
# ACTION ENUM
# -------------------------------------------------------------
class Action(Enum):
    FOLLOW_OWNER_HIGH_VEL = "FOLLOW_OWNER_HIGH_VEL"
    FOLLOW_OWNER_MID_VEL = "FOLLOW_OWNER_MID_VEL"
    FOLLOW_STRANGER_LOW_VEL = "FOLLOW_STRANGER_LOW_VEL"
    WAIT_AT_DOOR = "WAIT_AT_DOOR"
    PICK_TARGET = "pick_target"
    DELIVER_TO_OWNER = "DELIVER_TO_OWNER"
    PLACE_TARGET = "place_target"
    IDLE = "IDLE"


# -------------------------------------------------------------
# FUZZY SET
# -------------------------------------------------------------
class FuzzySet:
    """Simple triangular fuzzy set in meters."""
    def __init__(self, name, left, peak, right):
        self.name = name
        self.left = left
        self.peak = peak
        self.right = right

    def membership(self, x):
        if x <= self.left or x >= self.right:
            return 0.0
        elif x <= self.peak:
            return (x - self.left) / (self.peak - self.left)
        else:
            return (self.right - x) / (self.right - self.peak)


# -------------------------------------------------------------
# CONTROLLER WITH RL LAYER
# -------------------------------------------------------------
class RL_FuzzyController(Node):
    def __init__(self):
        super().__init__('rl_fuzzy_controller')

        # ---------------------------
        # Subscribers
        # ---------------------------
        self.create_subscription(Pose2D, '/robot_position', self.robot_callback, 10)
        self.create_subscription(Pose2D, '/toy_pose', self.toy_callback, 10)
        self.create_subscription(Pose2D, '/owner_position', self.owner_callback, 10)
        self.create_subscription(Pose2D, '/stranger_position', self.stranger_callback, 10)

        # RL feedback from motion controller or test
        self.create_subscription(String, '/action_feedback', self.feedback_callback, 10)

        # ---------------------------
        # Publisher
        # ---------------------------
        self.action_pub = self.create_publisher(String, '/current_action', 10)

        # ---------------------------
        # State variables
        # ---------------------------
        self.robot_pose = None
        self.toy_pose = None
        self.owner_pose = None
        self.stranger_pose = None

        self.is_holding_toy = False
        self.last_action = None
        self.last_action_for_rl = None
        self.last_action_time = time.time()

        # ---------------------------
        # FUZZY SETS (meters)
        # ---------------------------
        self.distance_sets = [
            FuzzySet("very_close", 0.00, 0.00, 0.50),
            FuzzySet("close", 0.30, 0.80, 1.30),
            FuzzySet("medium", 1.00, 1.80, 2.60),
            FuzzySet("far", 2.30, 3.00, 4.00),
            FuzzySet("very_far", 3.50, 4.00, 4.50),
        ]

        # ---------------------------
        # RULE DEFINITIONS
        # ---------------------------
        self.rules = self.define_rules()

        # ---------------------------
        # RL weights (initial = 1.0 = neutral)
        # ---------------------------
        self.learning_rates = {a: 0.1 for a in Action}
        self.learned_weights = {a: 1.0 for a in Action}

        # ---------------------------
        # Decision timer
        # ---------------------------
        self.create_timer(0.3, self.decision_loop)

        self.get_logger().info("RL Fuzzy Controller initialized.")

    # ---------------------------------------------------------
    # RULE SET
    # ---------------------------------------------------------
    def define_rules(self):
        return [
            {
                'conditions': [('toy', 'very_close')],
                'action': Action.PICK_TARGET,
                'weight': 1.0
            },
            {
                'conditions': [('owner', 'far'), ('owner', 'very_far')],
                'action': Action.FOLLOW_OWNER_HIGH_VEL,
                'weight': 0.9
            },
            {
                'conditions': [('owner', 'medium')],
                'action': Action.FOLLOW_OWNER_MID_VEL,
                'weight': 0.8
            },
            {
                'conditions': [('stranger', 'medium'), ('owner', 'very_far')],
                'action': Action.FOLLOW_STRANGER_LOW_VEL,
                'weight': 0.7
            },
            {
                'conditions': [('owner', 'close')],
                'action': Action.DELIVER_TO_OWNER,
                'condition_check': lambda: self.is_holding_toy,
                'weight': 1.0
            },
            {
                'conditions': [('owner', 'very_far')],
                'action': Action.PLACE_TARGET,
                'condition_check': lambda: self.is_holding_toy,
                'weight': 0.6
            },
            {
                'conditions': [('owner', 'very_far'), ('stranger', 'very_far')],
                'action': Action.WAIT_AT_DOOR,
                'weight': 0.5
            },
            {
                'conditions': [],
                'action': Action.IDLE,
                'weight': 0.1
            },
        ]

    # ---------------------------------------------------------
    # CALLBACKS FOR POSITION
    # ---------------------------------------------------------
    def robot_callback(self, msg): self.robot_pose = msg
    def toy_callback(self, msg): self.toy_pose = msg
    def owner_callback(self, msg): self.owner_pose = msg
    def stranger_callback(self, msg): self.stranger_pose = msg

    # ---------------------------------------------------------
    # DISTANCE
    # ---------------------------------------------------------
    def calculate_distance(self, p1, p2):
        if p1 is None or p2 is None:
            return float('inf')
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    # ---------------------------------------------------------
    # FUZZIFY
    # ---------------------------------------------------------
    def fuzzify_distance(self, dist):
        dist = min(dist, 4.5)
        return {fs.name: fs.membership(dist) for fs in self.distance_sets}

    # ---------------------------------------------------------
    # RULE EVALUATION (WITH RL)
    # ---------------------------------------------------------
    def evaluate_rule(self, rule, fuzzy):
        action = rule['action']

        # Conditions like "must be holding the toy"
        if 'condition_check' in rule:
            if not rule['condition_check']():
                return 0.0

        # No conditions: IDLE
        if not rule['conditions']:
            base = rule['weight']
            return base * self.learned_weights[action]

        # AND-logic (minimum)
        strength = 1.0
        for obj, name in rule['conditions']:
            if name not in fuzzy[obj]:
                return 0.0
            strength = min(strength, fuzzy[obj][name])

        base = rule['weight']
        mult = self.learned_weights[action]

        return strength * base * mult

    # ---------------------------------------------------------
    # MAIN DECISION LOGIC
    # ---------------------------------------------------------
    def decide_action(self):

        if None in [self.robot_pose, self.toy_pose, self.owner_pose, self.stranger_pose]:
            return Action.IDLE

        d_toy = self.calculate_distance(self.robot_pose, self.toy_pose)
        d_owner = self.calculate_distance(self.robot_pose, self.owner_pose)

        # Hard pickup rule
        if not self.is_holding_toy and d_toy < 0.30:
            self.is_holding_toy = True
            return Action.PICK_TARGET

        fuzzy = {
            'toy': self.fuzzify_distance(d_toy),
            'owner': self.fuzzify_distance(d_owner),
            'stranger': self.fuzzify_distance(
                self.calculate_distance(self.robot_pose, self.stranger_pose)
            )
        }

        scores = {}
        for rule in self.rules:
            score = self.evaluate_rule(rule, fuzzy)
            if score > 0:
                act = rule['action']
                scores[act] = max(scores.get(act, 0), score)

        if not scores:
            return Action.IDLE

        # Choose highest
        best_action = max(scores.items(), key=lambda x: x[1])[0]

        # Delivery success condition
        if best_action == Action.DELIVER_TO_OWNER and d_owner < 0.5:
            self.is_holding_toy = False

        return best_action

    # ---------------------------------------------------------
    # RL FEEDBACK CALLBACK
    # ---------------------------------------------------------
    def feedback_callback(self, msg):
        if self.last_action_for_rl is None:
            return

        feedback = msg.data.strip().upper()
        reward = 1.0 if feedback == "SUCCESS" else -1.0

        self.update_rule_weight(self.last_action_for_rl, reward)

    def update_rule_weight(self, action, reward):
        lr = self.learning_rates[action]
        old = self.learned_weights[action]

        target = 1.0 + reward   # 0, 1, or 2
        new = old + lr * (target - old)
        new = max(0.3, min(2.0, new))

        self.learned_weights[action] = new

        self.get_logger().info(
            f"[RL] {action.value}: reward={reward}, {old:.2f}→{new:.2f}"
        )

    # ---------------------------------------------------------
    # DECISION LOOP
    # ---------------------------------------------------------
    def decision_loop(self):
        if self.robot_pose is None:
            return

        action = self.decide_action()

        # RL needs to know what we chose
        self.last_action_for_rl = action

        # Publish only if changed
        if action != self.last_action:
            msg = String()
            msg.data = action.value
            self.action_pub.publish(msg)
            self.get_logger().info(f"Action: {action.value}")
            self.last_action = action
            self.last_action_time = time.time()


# -------------------------------------------------------------
# MAIN
# -------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RL_FuzzyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

