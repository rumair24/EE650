#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
 
import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
 
###############################################################################
# ZeroState: Publishes zero velocities for a fixed duration (used for Start/Stop)
###############################################################################
class ZeroState(State):
    def __init__(self, publisher, duration: float, description: str):
        super().__init__(["done"])
        self.publisher = publisher
        self.twist = Twist()  # all zeros
        self.duration = duration
        self.description = description
 
    def execute(self, blackboard: Blackboard) -> str:
        print(f"{self.description} for {self.duration} seconds.")
        start_time = time.time()
        while time.time() - start_time < self.duration:
            self.publisher.publish(self.twist)
            time.sleep(0.1)
        return "done"
 
###############################################################################
# TimedMoveState: Publishes a Twist command for a fixed duration
###############################################################################
class TimedMoveState(State):
    def __init__(self, publisher, twist: Twist, duration: float, description: str):
        super().__init__(["done"])
        self.publisher = publisher
        self.twist = twist
        self.duration = duration
        self.description = description
 
    def execute(self, blackboard: Blackboard) -> str:
        print(f"{self.description} for {self.duration} seconds.")
        start_time = time.time()
        while time.time() - start_time < self.duration:
            self.publisher.publish(self.twist)
            time.sleep(0.1)
        return "done"
 
###############################################################################
# SquareFSM: Executes a square trajectory using forward movement and 90° turns
###############################################################################
class SquareFSM(StateMachine):
    def __init__(self, publisher):
        super().__init__(outcomes=["done"])
        # Define forward twist
        twist_forward = Twist()
        twist_forward.linear.x = 0.2
 
        # Define turning twist (approx. 90° turn)
        twist_turn = Twist()
        twist_turn.angular.z = 1.0
 
        # Add states for each side and turn of the square
        self.add_state("MOVE1", TimedMoveState(publisher, twist_forward, 2.0, "Square: Moving forward (side 1)"),
                       transitions={"done": "TURN1"})
        self.add_state("TURN1", TimedMoveState(publisher, twist_turn, 1.57, "Square: Turning 90° (side 1)"),
                       transitions={"done": "MOVE2"})
        self.add_state("MOVE2", TimedMoveState(publisher, twist_forward, 2.0, "Square: Moving forward (side 2)"),
                       transitions={"done": "TURN2"})
        self.add_state("TURN2", TimedMoveState(publisher, twist_turn, 1.57, "Square: Turning 90° (side 2)"),
                       transitions={"done": "MOVE3"})
        self.add_state("MOVE3", TimedMoveState(publisher, twist_forward, 2.0, "Square: Moving forward (side 3)"),
                       transitions={"done": "TURN3"})
        self.add_state("TURN3", TimedMoveState(publisher, twist_turn, 1.57, "Square: Turning 90° (side 3)"),
                       transitions={"done": "MOVE4"})
        self.add_state("MOVE4", TimedMoveState(publisher, twist_forward, 2.0, "Square: Moving forward (side 4)"),
                       transitions={"done": "TURN4"})
        self.add_state("TURN4", TimedMoveState(publisher, twist_turn, 1.57, "Square: Turning 90° (side 4)"),
                       transitions={"done": "done"})
        self.set_start_state("MOVE1")
 
###############################################################################
# MasterFSM: Combines Start, Square, and Stop states into one cycle.
###############################################################################
class MasterFSM(StateMachine):
    def __init__(self, publisher):
        super().__init__(outcomes=["done"])
        # Create a start state (send zero velocities)
        self.add_state("START", ZeroState(publisher, duration=1.0, description="Starting (zero velocity)"),
                       transitions={"done": "SQUARE"})
        # Add the square movement as a sub-state machine
        self.add_state("SQUARE", SquareFSM(publisher), transitions={"done": "STOP"})
        # Create a stop state (send zero velocities)
        self.add_state("STOP", ZeroState(publisher, duration=1.0, description="Stopping (zero velocity)"),
                       transitions={"done": "done"})
        self.set_start_state("START")
 
###############################################################################
# Main: Initialize ROS 2, run the MasterFSM, and optionally loop cycles.
###############################################################################
def main():
    rclpy.init()
    set_ros_loggers()
    node = rclpy.create_node("turtlebot_square_master")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
 
    # Shared blackboard (not used in this simple example)
    blackboard = Blackboard()
    blackboard["node"] = node
 
    master_fsm = MasterFSM(publisher)
    # Instantiate Yasmin Viewer to visualize the MasterFSM
    YasminViewerPub("turtlebot_square_master", master_fsm)
 
    node.get_logger().info("Starting TurtleBot Square Movement Cycle (Start -> Square -> Stop)")
 
    try:
        while rclpy.ok():
            outcome = master_fsm(blackboard)
            node.get_logger().info(f"Master FSM cycle completed with outcome: {outcome}")
            # Optional: add a pause between cycles if you want to repeat the cycle
            time.sleep(1.0)
    except KeyboardInterrupt:
        node.get_logger().info("Master FSM interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == "__main__":
    main()