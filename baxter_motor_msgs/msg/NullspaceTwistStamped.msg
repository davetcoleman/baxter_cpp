# A twist to apply to the endpoint of the robot
geometry_msgs/TwistStamped     twist_stamped

# A nullspace position setpoint for the arm joint angles
#   this defines the nullspace pose that the arm will be
#   driven towards during all Cartesian twist commands.
#   Note that the nullspace is only capable of generating
#   internal self-motions of a kinematic chain - so this
#   can be thought of as a preferred pose to bias the arm
#   toward.
MotorControlMsgs/JointPosition nullspace_setpoint