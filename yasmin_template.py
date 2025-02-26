#!/usr/bin/env python3
"""
YASMIN State Machine Template
 
Instructions:
1. Define your states by creating classes that inherit from yasmin.State.
   - Each state should define its possible outcomes (e.g., "next", "error", "end").
   - Implement the execute() method where you define the state’s behavior.
2. Use a Blackboard (shared data storage) to pass data between states.
3. Create a StateMachine instance, add your states, and define the transitions.
4. Optionally, use the YasminViewerPub to visualize the state machine in a web viewer.
5. Run the state machine, then analyze the final outcome.
 
Make sure ROS 2 is set up in your environment before running this code.
"""
 
import time
import rclpy
from rclpy.node import Node
 
# Import YASMIN modules
import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
 
# ====================================================
# Step 1: Define Your States
# ====================================================
 
class StateA(State):
    """
    Example State A:
    - Outcome "to_B": Transition to StateB.
    - Outcome "to_end": Exit the state machine.
    """
    def __init__(self) -> None:
        super().__init__(outcomes=["to_B", "to_end"])
        # Initialize state-specific variables if needed
 
    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing State A")
        # [Instruction] Add your state behavior here.
        # For example, perform some actions and update the blackboard:
        # blackboard["data"] = "value"
        time.sleep(1)  # Simulate some work
        # [Instruction] Determine the outcome based on your logic:
        # Return "to_B" to transition to StateB or "to_end" to finish.
        return "to_B"
 
 
class StateB(State):
    """
    Example State B:
    - Outcome "to_A": Transition back to StateA.
    - Outcome "to_end": Exit the state machine.
    """
    def __init__(self) -> None:
        super().__init__(outcomes=["to_A", "to_end"])
 
    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing State B")
        # [Instruction] Add your state behavior here.
        time.sleep(1)
        # [Instruction] Use conditions to decide which outcome to return.
        return "to_A"
 
 
# ====================================================
# Step 2: Build and Configure the State Machine
# ====================================================
 
def main():
    # Initialize ROS 2 and set up ROS loggers for debugging.
    rclpy.init()
    set_ros_loggers()
 
    # Create an instance of a StateMachine.
    # The outcomes provided here are the final outcomes of the state machine (e.g., "end").
    sm = StateMachine(outcomes=["end"])
 
    # Add states to the state machine with their corresponding transitions.
    # The transitions dict maps state outcomes to the name of the next state.
    sm.add_state("StateA", StateA(), transitions={"to_B": "StateB", "to_end": "end"})
    sm.add_state("StateB", StateB(), transitions={"to_A": "StateA", "to_end": "end"})
 
    # [Instruction] Optionally set the starting state (if not, the first added state is used).
    sm.set_start_state("StateA")
 
    # Optional: Launch the YASMIN web viewer to visualize state machine execution.
    YasminViewerPub("MyStateMachine", sm)
 
    # Create a new blackboard instance to share data between states.
    blackboard = Blackboard()
 
    # ====================================================
    # Step 3: Execute the State Machine
    # ====================================================
    try:
        final_outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO("State machine completed with outcome: " + final_outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_ERROR("Error during state machine execution: " + str(e))
    finally:
        # Shutdown ROS 2 when finished.
        rclpy.shutdown()
 
if __name__ == "__main__":
    main()