Header header

#flags and status
int32[] overPosConfigFlags
int32[] overVelConfigFlags
int32[] overJointLimitFlags
bool armIsStalled

#binary mask
uint8 RESET = 0
uint8 VALID = 1                        #solution valid
uint8 JOINT_LIMIT_ZOH = 2              #joint limit exceeded
uint8 OVER_JOINT_POSITION_ZOH = 4      #large difference between actual and commanded positions on at least one joint
uint8 OVER_JOINT_VELOCITY_ZOH = 8      #large commanded joint angular velocity on at least once joint
uint8 OVER_TWIST_ERROR_MAGNITUDE_ZOH = 16   #large twist error when commanded twist is small
uint8 ALL_SOLVERS_FAILED = 32          #no solution found
