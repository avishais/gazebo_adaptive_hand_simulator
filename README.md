# Gazebo underactuated hand simulator

The simulator offers three hands: Reflex, Model-O and Model-T42 (Yale OpenHand project - https://www.eng.yale.edu/grablab/openhand/). To switch between the hands:
    1. Set the xacro filename in hand.launch to either hand_T42.xacro, hand_O.xacro or hand_reflex.hand.
    2. Set the 'is_model_O_reflex' argument in hand_control.launch to 'true' for the reflex/model-O or 'false' for model-T42.
    3. In gripper.yaml, 'set gripper_type' name.
    
To run the simulation:
    1. roslaunch hand_simulator hand.launch (Loads the environment)
    2. roslaunch hand_simulator hand_control.launch (Loads the controller)
    3. rosrun hand_simulator keyboard_control.py (Loads keyboard control - tested only on the Model-T42).
