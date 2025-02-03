package frc.robot.subsystems.arm;


import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.hardware.CANdi;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ArmSubsystem {


    private final SparkFlex m_shoulderMotor;
    private final SparkMax m_wristMotor;
    private final CANdi m_armCANdi;
    private ArmState m_shoulderState = new ArmState(
        Angle.ofBaseUnits(0.0, Degree), 
        AngularVelocity.ofBaseUnits(0.0, DegreesPerSecond));
    private ArmState m_wristState = new ArmState(
        Angle.ofBaseUnits(0.0, Degree), 
        AngularVelocity.ofBaseUnits(0.0, DegreesPerSecond));
    

public ArmSubsystem() {

    m_shoulderMotor = new SparkFlex(18, MotorType.kBrushless);
    m_wristMotor = new SparkMax(19, MotorType.kBrushless);
    m_armCANdi = new CANdi(34);
    }

public ArmState getShoulderState() {
    Angle shoulderAngle = m_armCANdi.getPWM1Position().getValue();
    AngularVelocity shoulderVelocity = m_armCANdi.getPWM1Velocity().getValue();
    m_shoulderState.setArmState(shoulderAngle, shoulderVelocity); 
    return m_shoulderState;
    }

public ArmState getWristState() {
    Angle wristAngle = m_armCANdi.getPWM2Position().getValue();
    AngularVelocity wristVelocity = m_armCANdi.getPWM2Velocity().getValue();
    m_wristState.setArmState(wristAngle, wristVelocity); 
    return m_wristState;
    }
}
