# 2025-Robot

### CAN ID Configuration ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| roborio | core | 0-9 | master (no ID) |
| pdh | core | 0-9 | 0
| front_left_drive | CANSparkMax | 10-29 | 10 |
| front_left_steer | CANSparkMax | 10-29 | 11 |
| front_right_drive | CANSparkMax | 10-29 | 12 |
| front_right_steer | CANSparkMax | 10-29 | 13 |
| back_left_drive | CANSparkMax | 10-29 | 14 |
| back_left_steer | CANSparkMax | 10-29 | 15 |
| back_right_drive | CANSparkMax | 10-29 | 16 |
| back_right_steer | CANSparkMax | 10-29 | 17 |


### Digital IO ID Configuration ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| front_left_encoder | DutyCycleEncoder | 0-9 | 0 |
| front_right_encoder | DutyCycleEncoder | 0-9 | 1 |
| back_left_encoder | DutyCycleEncoder | 0-9 | 2 |
| back_right_encoder | DutyCycleEncoder | 0-9 | 3 |

### Analog Input ID Configuration ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| high_pressure_transducer | AnalogInput | 0-3 | 0 |
| working_pressure_transducer | AnalogInput | 0-3 | 1 |