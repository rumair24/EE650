#!/usr/bin/env python3
import time
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

# ---------------------------------------------------------------------------
# A simple state that waits for keyboard input.
# It returns "move" when the user types "m" and "stop" when the user types "s".
# ---------------------------------------------------------------------------
class TeleopState(State):
    def __init__(self, prompt, key_command_map):
        outcomes = list(key_command_map.values()) + ["invalid"]
        super().__init__(outcomes)
        self.prompt = prompt
        self.key_command_map = key_command_map

    def execute(self, blackboard: Blackboard) -> str:
        cmd = input(self.prompt).strip().lower()
        if cmd in self.key_command_map:
            return self.key_command_map[cmd]
        else:
            print("Invalid command, try again.")
            return "invalid"

# ---------------------------------------------------------------------------
# A state that publishes a Twist message. It is parameterized by a Twist command
# and a description. (Used by movement states.)
# ---------------------------------------------------------------------------
class MoveState(State):
    def __init__(self, publisher, twist: Twist, description: str):
        super().__init__(["done"])
        self.publisher = publisher
        self.twist = twist
        self.description = description

    def execute(self, blackboard: Blackboard) -> str:
        self.publisher.publish(self.twist)
        print(f"{self.description}: {self.twist}")
        time.sleep(1)
        return "done"

# ---------------------------------------------------------------------------
# A state to publish a stop command (Twist message with zeros).
# ---------------------------------------------------------------------------
class StopState(State):
    def __init__(self, publisher):
        super().__init__(["done"])
        self.publisher = publisher
        self.twist = Twist()  # defaults are zeros

    def execute(self, blackboard: Blackboard) -> str:
        self.publisher.publish(self.twist)
        print("Stop command published.")
        time.sleep(1)
        return "done"

# ---------------------------------------------------------------------------
# The following states add complexity to the nested (complex) movement FSM.
# They demonstrate decisions and looping that would be awkward in a linear script.
# ---------------------------------------------------------------------------

class InitializeCounterState(State):
    def __init__(self):
        super().__init__(["done"])
    def execute(self, blackboard: Blackboard) -> str:
        blackboard["pattern_count"] = 0
        print("Counter initialized to 0")
        return "done"

class DecidePatternState(State):
    def __init__(self):
        super().__init__(outcomes=["pattern_a", "pattern_b"])
    def execute(self, blackboard: Blackboard) -> str:
        # Simulate a decision based on a random condition.
        val = random.random()
        if val < 0.5:
            print("DecidePattern: Choosing Pattern A")
            return "pattern_a"
        else:
            print("DecidePattern: Choosing Pattern B")
            return "pattern_b"

class IncrementCounterState(State):
    def __init__(self):
        super().__init__(["done"])

    def execute(self, blackboard: Blackboard) -> str:
        # Initialize 'pattern_count' if it doesn't exist
        if not hasattr(blackboard, 'pattern_count'):
            blackboard.pattern_count = 0
        # Increment the counter
        blackboard.pattern_count += 1
        print(f"Counter incremented to {blackboard.pattern_count}")
        return "done"

class LoopCheckState(State):
    def __init__(self, max_loops: int):
        super().__init__(outcomes=["loop", "finish"])
        self.max_loops = max_loops

    def execute(self, blackboard: Blackboard) -> str:
        # Ensure 'pattern_count' is initialized
        if not hasattr(blackboard, 'pattern_count'):
            blackboard.pattern_count = 0
        count = blackboard.pattern_count
        print(f"LoopCheck: current count = {count}")
        if count < self.max_loops:
            return "loop"
        else:
            return "finish"

# ---------------------------------------------------------------------------
# The nested ComplexMovementFSM.
# It first initializes a counter, then in each cycle decides between two patterns:
#
# Pattern A:
#   - MOVE_FORWARD_A
#   - TURN_LEFT_A
#   - MOVE_FORWARD_A2
#
# Pattern B:
#   - MOVE_BACKWARD_B
#   - TURN_RIGHT_B
#   - MOVE_FORWARD_B
#   - TURN_LEFT_B
#
# After running a pattern, it increments a counter and checks if it should loop.
# This complexity (conditional branching and looping) is best modeled with a state machine.
# ---------------------------------------------------------------------------
class ComplexMovementFSM(StateMachine):
    def __init__(self, publisher):
        super().__init__(outcomes=["finished"])
        self.publisher = publisher

        # Define twist messages
        twist_forward = Twist()
        twist_forward.linear.x = 0.3

        twist_left = Twist()
        twist_left.angular.z = 0.5

        twist_right = Twist()
        twist_right.angular.z = -0.5

        twist_backward = Twist()
        twist_backward.linear.x = -0.3

        # Create states for Pattern A
        move_forward_a = MoveState(publisher, twist_forward, "Pattern A: Move Forward")
        turn_left_a = MoveState(publisher, twist_left, "Pattern A: Turn Left")
        move_forward_a2 = MoveState(publisher, twist_forward, "Pattern A: Move Forward Again")

        # Create states for Pattern B
        move_backward_b = MoveState(publisher, twist_backward, "Pattern B: Move Backward")
        turn_right_b = MoveState(publisher, twist_right, "Pattern B: Turn Right")
        move_forward_b = MoveState(publisher, twist_forward, "Pattern B: Move Forward")
        turn_left_b = MoveState(publisher, twist_left, "Pattern B: Turn Left")

        decide_pattern = DecidePatternState()
        inc_counter = IncrementCounterState()
        loop_check = LoopCheckState(max_loops=2)

        # Add states and transitions:
        self.add_state("INIT_COUNTER", InitializeCounterState(), transitions={"done": "DECIDE_PATTERN"})
        self.add_state("DECIDE_PATTERN", decide_pattern, transitions={"pattern_a": "PATTERN_A",
                                                                      "pattern_b": "PATTERN_B"})
        # Pattern A branch
        self.add_state("PATTERN_A", move_forward_a, transitions={"done": "TURN_LEFT_A"})
        self.add_state("TURN_LEFT_A", turn_left_a, transitions={"done": "MOVE_FORWARD_A2"})
        self.add_state("MOVE_FORWARD_A2", move_forward_a2, transitions={"done": "INCREMENT"})
        # Pattern B branch
        self.add_state("PATTERN_B", move_backward_b, transitions={"done": "TURN_RIGHT_B"})
        self.add_state("TURN_RIGHT_B", turn_right_b, transitions={"done": "MOVE_FORWARD_B"})
        self.add_state("MOVE_FORWARD_B", move_forward_b, transitions={"done": "TURN_LEFT_B"})
        self.add_state("TURN_LEFT_B", turn_left_b, transitions={"done": "INCREMENT"})
        # Common states for counter and loop check
        self.add_state("INCREMENT", inc_counter, transitions={"done": "LOOP_CHECK"})
        self.add_state("LOOP_CHECK", loop_check, transitions={"loop": "DECIDE_PATTERN", "finish": "finished"})

        self.set_start_state("INIT_COUNTER")

# ---------------------------------------------------------------------------
# The main FSM that waits for teleop input.
# Press "m" to run the complex nested movement FSM.
# Press "s" to issue an immediate stop command.
# ---------------------------------------------------------------------------
class MainFSM(StateMachine):
    def __init__(self, publisher):
        super().__init__(outcomes=["exit"])
        key_map = {"m": "move", "s": "stop"}
        teleop = TeleopState("Enter command (m: complex movement, s: stop): ", key_map)
        stop_state = StopState(publisher)
        complex_movement_fsm = ComplexMovementFSM(publisher)

        self.add_state("TELEOP", teleop, transitions={"move": "MOVEMENT",
                                                       "stop": "STOP",
                                                       "invalid": "TELEOP"})
        self.add_state("MOVEMENT", complex_movement_fsm, transitions={"finished": "TELEOP"})
        self.add_state("STOP", stop_state, transitions={"done": "TELEOP"})
        self.set_start_state("TELEOP")

# ---------------------------------------------------------------------------
# Main function: initializes ROS 2, creates a publisher to /cmd_vel,
# and then runs the main FSM.
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    set_ros_loggers()
    node = rclpy.create_node("complex_teleop_fsm_node")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)

    fsm = MainFSM(publisher)
    
    # (Optional) Start the YASMIN web viewer:
    YasminViewerPub("complex_teleop_fsm", fsm)

    
    print("Starting complex teleop FSM. Use 'm' for complex movement and 's' for stop.")
    try:
        while rclpy.ok():
            outcome = fsm()
            print(f"FSM cycle completed with outcome: {outcome}")
    except KeyboardInterrupt:
        print("FSM interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
