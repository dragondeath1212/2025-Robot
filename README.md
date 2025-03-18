# 2025-Robot

### CAN ID Configuration ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| roborio | core | 0-9 | master (no ID) |
| pdh | core | 0-9 | 0
| front_left_drive | SparkMax | 10-29 | 10 |
| front_left_steer | SparkMax | 10-29 | 11 |
| front_right_drive | SparkMax | 10-29 | 12 |
| front_right_steer | SparkMax | 10-29 | 13 |
| back_left_drive | SparkMax | 10-29 | 14 |
| back_left_steer | SparkMax | 10-29 | 15 |
| back_right_drive | SparkMax | 10-29 | 16 |
| back_right_steer | SparkMax | 10-29 | 17 |
| shoulder_motor | SparkFlex | 10-29 | 18 | 
| wrist_motor | SparkMax | 10-29 | 19 |
| left_intake_motor | SparkMax | 10-29 | 20 | 
| right_intake_motor | SparkMax | 10-29 | 21 |
| elevator_motor | SparkFlex | 10-29 | 22 |
| front_left_encoder | CANCoder | 30-39 | 30 |
| front_right_encoder | CANCoder | 30-39 | 31 |
| back_left_encoder | CANCoder | 30-39 | 32 |
| back_right_encoder | CANCoder | 30-39 | 33 |
| arm_candi | CANdi | 30-39 | 34 |


### Analog Input ID Configuration ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| high_pressure_transducer | AnalogInput | 0-3 | 0 |
| working_pressure_transducer | AnalogInput | 0-3 | 1 |


### Digital Input ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| elevator_encoder | DutyCycleEncoder | 0-3 | 0 |

### Solenoids ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| left_intake_solenoid | DoubleSolenoid | 0-15 | 0, 1 |
| right_intake_solenoid | DoubleSolenoid | 0-15 | 2, 3 |
| left_intake_solenoid | DoubleSolenoid | 0-15 | 0, 1 |

### Drive Controls ###
- **Left Stick** - field oriented robot movement
  - *Partially implemented*
  - Should verify that the orientation is such that when standing at the driver station, up is towards the opposing alliance, down is towards our alliance, left/right are relative to the driver.
- **Right Stick** - field oriented robot rotation
  - *Needs rework*
  - Currently left on the stick rotates the robot left, right rotates the robot right.  I think it would be nice
  to have this field oriented as well, so that pushing a direction on the stick rotates the robot to face in that direction (e.g. up on the stick, robot turns to face opposing alliance, down on the stick, robot turns to face the driver).
- **Left Trigger/Right trigger** - robot oriented rotation
  - left trigger rotates the robot counterclockwise, right trigger rotates the robot clockwise
- **D-Pad** - fine grained robot oriented movement
  - Up on the d-pad moves the robot forward, down backward, etc.
- **A button** - auto drive to nearest scoring position
  - *Partially implemented*
  - Use the path planner to drive the robot to the nearest reef scoring position, and rotate the bot to
  face the april tag at that position.
  - Currently it is always taking you to the nearest scoring position.  Can we use the switches on the drive station to control which locations have already been scored, and then filter those out from the "nearest target" calculation?
- **B button** - stop the robot
  - Stops the robot, interrupting any running commands
- **X button** - unused
- **Left Bumper** - auto drive to left loading station
  - Auto generate a path to take the robot to the left loading station (drivers left hand side), rotated so that the bot is ready to be loaded
  - "alliance aware", meaning left and right depends on which side of the field you are one.
- **Right Bumper** - auto drive to right loading station
  - Auto generate a path to take the robot to the right loading station (drivers right hand side), rotated so that the bot is ready to be loaded
  - "alliance aware", meaning left and right depends on which side of the field you are one.
