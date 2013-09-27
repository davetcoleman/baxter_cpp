Header header
int32 Timestamp
int32 LastTimestamp
int32 Position
int32 RawPosition
int32 Velocity
int32 Acceleration

int32 Current
# 
int32 Torque0 # tared torque from 0th sea assembly
int32 RawHes0 # raw hes values form 0th sea
int32 Torque1
int32 RawHes1
int32 MeasuredTorque # the "final" computed value for joint torque from hes sensors = (Torque0 + Torque1)/2

int32 LoadCellValue
int32 MotorTemp
int32 MotorTempRaw
int32 HeatsinkTemp

int32 Duty
int32 isEnabled # if joint is enabled 
int32 isMotorEnabled
int32 isFunctionOn

# function generator values
int32 Period
int32 LowerLimit
int32 UpperLimit
int32 FunctionMode
int32 ControlMode

# stuff for torque controller
int32 RefTorque

int32 RefPosition

float32 GravityTorque
float32 GravitySpringTorque

int32 MotorPosition
int32 MotorVelocity

int32 ErrorCode
int32 LatchedErrorCode
int32 BoardErrorCode
int32 MissedFrames
int32 Flags
int32 BoardFlags

float32 CyclicPosition
float32 CyclicEffort
float32 CyclicADC
float32 CyclicCurrent
float32 CyclicVelocity

float32 LoadCellCalibrated

