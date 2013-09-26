int32[] GainP
int32[] GainI
int32[] GainD
int32[] GainFF
int32[] GainBEMF
int32[] GainAlpha

#int32 Zero
#int32 SensorDeadband
#int32 UpperLimit		# clockwise limit for the motor
#int32 LowerLimit		# counterclockwise limit for the motor
#int32[] Deadband
#int32 SlewRate

  #used internally
#int32 Error			# error between commanded and actual
#int32 ErrorOld			# old error (for derivative control)
#int32 Accumulator		# accumulator for integral term
#int32 AntiWindup
#int32 ActualOld
#int32 DesiredOld
#int32 DervalOld
#int32 PropvalOld

#int32 FilterOutput
#int32 Mode