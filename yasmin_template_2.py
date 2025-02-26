#!/usr/bin/env python3
"""
YASMIN General State Machine Template
---------------------------------------
This template demonstrates how to design a state machine using the YASMIN library in ROS 2.
It is a general-purpose example that shows state transitions and the use of a shared blackboard.
 
Instructions:
1. Each state is implemented as a subclass of `yasmin.State` with an `execute(blackboard)` method.
2. Transitions are defined by mapping outcomes to the next state's name.
3. A shared blackboard is used to pass data between states.
4. The Yasmin Viewer is used to visualize state transitions.
5. Customize the states (e.g., initialization, processing, and finalization) to suit your application.
 
Run this template to see how states change from one to another.
"""
 
import time
import rclpy
from rclpy.node import Node
 
# Import YASMIN components
import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
 
##############################################
# Step 1: Define your State classes
##############################################
class InitState(State):
    """
    InitState: Performs initialization tasks.
    Outcome: "done" indicates completion of initialization.
    """
    def __init__(self):
        super().__init__(["done"])
    def execute(self, blackboard: Blackboard) -> str:
        print("InitState: Initializing system...")
        # Simulate initialization work
        time.sleep(1.0)
        # Set an initial counter on the blackboard (example shared data)
        blackboard["counter"] = 0
        return "done"
 
class ProcessState(State):
    """
    ProcessState: Represents a generic processing task.
    It increments a counter and loops until a threshold is reached.
    Outcomes:
      - "increment": Continue processing.
      - "complete": Processing finished.
    """
    def __init__(self):
        super().__init__(["increment", "complete"])
    def execute(self, blackboard: Blackboard) -> str:
        # Retrieve the counter from the blackboard, defaulting to 0 if not present
        if "counter" in blackboard:
            counter = blackboard["counter"]
        else:
            counter = 0
        print(f"ProcessState: Processing... Current counter = {counter}")
        # Simulate processing work: increment the counter
        counter += 1
        blackboard["counter"] = counter
        time.sleep(1.0)
        # Decide next outcome based on counter value
        if counter < 3:
            print("ProcessState: Incrementing counter and continuing processing.")
            return "increment"
        else:
            print("ProcessState: Processing complete.")
            return "complete"
 
class FinalState(State):
    """
    FinalState: Finalizes the process and cleans up.
    Outcome: "done" indicates that the state machine cycle is complete.
    """
    def __init__(self):
        super().__init__(["done"])
    def execute(self, blackboard: Blackboard) -> str:
        print("FinalState: Finalizing and cleaning up...")
        # Simulate finalization work
        time.sleep(1.0)
        print("FinalState: Cycle complete.")
        return "done"
 
##############################################
# Step 2: Define your State Machine
##############################################
class GeneralStateMachine(StateMachine):
    """
    GeneralStateMachine: Demonstrates a simple sequence of state transitions.
    Flow:
      INIT -> PROCESS (loops until counter >= 3) -> FINAL
    """
    def __init__(self):
        super().__init__(outcomes=["done"])
        # Add states with their transitions:
        self.add_state("INIT", InitState(), transitions={"done": "PROCESS"})
        self.add_state("PROCESS", ProcessState(), transitions={"increment": "PROCESS", "complete": "FINAL"})
        self.add_state("FINAL", FinalState(), transitions={"done": "done"})
        # Set the initial state of the state machine
        self.set_start_state("INIT")
 
##############################################
# Step 3: Main function to run the state machine
##############################################
def main():
    # Initialize ROS 2 and set up logging
    rclpy.init()
    set_ros_loggers()
    node = rclpy.create_node("yasmin_general_state_machine")
    # Create a blackboard for sharing data between states
    blackboard = Blackboard()
    blackboard["node"] = node
    # Instantiate the state machine
    sm = GeneralStateMachine()
    # Optional: Instantiate Yasmin Viewer to visualize the state machine
    YasminViewerPub("yasmin_general_state_machine", sm)
    node.get_logger().info("Starting the general YASMIN state machine template...")
    # Run the state machine once (or loop for repeated cycles)
    try:
        outcome = sm(blackboard)
        node.get_logger().info(f"State machine cycle completed with outcome: {outcome}")
    except KeyboardInterrupt:
        node.get_logger().info("State machine interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == "__main__":
    main()