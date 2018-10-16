# Gazebo underactuated hand simulator

The simulator offers three hands: Reflex, Model-O and Model-T42 (Yale OpenHand project - https://www.eng.yale.edu/grablab/openhand/). 
    
To run the simulation:

    1. In gripper.yaml, set 'gripper_type' name ('reflex','model_O','model_T42').
    
    2. roslaunch hand_simulator hand.launch model:={hand_reflex,hand_O,'hand_T42'} (Loads the environment).
    
    2. roslaunch hand_simulator hand_control.launch model3:={true for reflex or O, false for T42} (Loads the controller).
    
    3. rosrun hand_simulator keyboard_control.py (Loads keyboard control - tested only on the Model-T42).
    
