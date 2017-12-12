import numpy as np
import RobotArmKinematics_v04.py
import Arm_EndPoint_Control_v05.py





def mainloop():

# Robot Arm used in ME439
    robot = merbt.make_me439_robot_arm()
    # Add a Marker: 
    robot.append(merbt.robot_link(robot[-1],rotation_axis=[0,0,0],endpoint_translation=[0.008,0,-0.035]))
    # Find the Free Axes of the robot
    free_axes = merbt.find_free_axes(robot)



    # Make and save the Servo angle to Command_Mappings
    angle_to_command_mappings = make_angle_to_command_mappings()
    
    # Set up the endpoint position that is "neutral"
    endpoint_position_target = np.array([0.08337574,  0., 0.14849218])
    # set up the joint angles version of current_servo_commands
	joint_angle_targets = np.radians(np.array([0.,-90.,90.,0.,0.,0.]) )
    servo_angle_targets = joint_angle_targets
    servo_angle_targets[2] = servo_angle_targets[2]+servo_angle_targets[1]  # Note that the 3rd element is now Beta2WORLD
    # and convert it to Servo commands
    servo_command_targets = map_servo_angles_to_commands(joint_angle_targets, angle_to_command_mappings)
    init_servo_command_targets = servo_command_targets

    # If there's a real robot, move it to neutral
    if real_robot:
        # Set up the servos
        servos = initialize_servos(servo_command_targets)
        # should now be at the target values, but move there again, because why not. 
        move_servos(servos,servo_command_targets) 




if __name__=="__main__":
    try:
        robot = mainloop()
    except KeyboardInterrupt :
        pass