cmake_minimum_required(VERSION 2.8.3)
project(MotorControlMsgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs geometry_msgs)

add_message_files(
  FILES
  AccelerometerStates.msg
  CollisionDetection.msg
  DebugVelocityController.msg
  GripperConfiguration.msg
  Inertia.msg
  PoseFixed.msg
  SplineProfileParameters.msg
  TwistFixed.msg
  WrenchLevel.msg
  AnalogIOState.msg
  Constraint.msg
  DigitalIOState.msg
  GripperIdentity.msg
  JointPosition.msg
  SEAJointState.msg
  StiffnessConstraint.msg
  TwistLevel.msg
  AnalogIOStates.msg
  DebugJointCommand.msg
  DigitalIOStates.msg
  GripperProperties.msg
  LimbState.msg
  SonarState.msg
  Stiffness.msg
  TwistOrient.msg
  AnalogOutputCommand.msg
  DebugJointState.msg
  DigitalOutputCommand.msg
  GripperState.msg
  LimbStates.msg
  SonarStates.msg
  StringArray.msg
  URDFSegment.msg
  AssemblyState.msg
  DebugNodeState.msg
  EndpointState.msg
  GripperStates.msg
  Navigator.msg
  SpeedConstraint.msg
  SuckerState.msg
  Vector3Fixed.msg
  AssemblyStates.msg
  DebugServoLoopStateDetail.msg
  EndpointStates.msg
  HeadPanCommand.msg
  NavigatorStates.msg
  SpeedLimit.msg
  SuckerStates.msg
  VelocityControllerState.msg
  CollisionAvoidanceState.msg
  DebugServoLoopState.msg
  GripperCommand.msg
  HeadState.msg
  NullspaceTwistStamped.msg
  SpeedLimitScale.msg
  TwistConstraint.msg
  WrenchConstraint.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs)
