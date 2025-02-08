package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.configs.PWM2Configs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.PWM;

import frc.robot.Constants.ArmConstants;

public class ArmEncoder {
    private final CANdi m_armCANdi;
    private final int m_signalId;
    private Angle m_position;
    private AngularVelocity m_velocity;
    private final PWM1Configs m_PWM1Config = new PWM1Configs();
    private final PWM2Configs m_PWM2Config = new PWM2Configs();


    public ArmEncoder(CANdi m_armCANdi, int signalId) {
        this.m_armCANdi = m_armCANdi;
        this.m_signalId = signalId;

        m_PWM1Config.withAbsoluteSensorDiscontinuityPoint(ArmConstants.SHOULDER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT);
        m_PWM2Config.withAbsoluteSensorDiscontinuityPoint(ArmConstants.WRIST_ABSOLUTE_SENSOR_DISCONTINUITY_POINT);

        m_PWM1Config.withAbsoluteSensorOffset(ArmConstants.SHOULDER_ABSOLUTE_SENSOR_OFFSET);
        m_PWM2Config.withAbsoluteSensorOffset(ArmConstants.WRIST_ABSOLUTE_SENSOR_OFFSET);

        m_PWM1Config.withSensorDirection(ArmConstants.SHOULDER_ENCODER_IS_INVERTED);
        m_PWM2Config.withSensorDirection(ArmConstants.WRIST_ENCODER_IS_INVERTED);
    }

    public Angle getPosition() {
        if (m_signalId == 1) {
            return m_armCANdi.getPWM1Position().getValue();
        }
        else {
            return m_armCANdi.getPWM2Position().getValue();
        }
    }

    public AngularVelocity getVelocity() {
        if (m_signalId == 1) {
            return m_armCANdi.getPWM1Velocity().getValue();
        }
        else {
            return m_armCANdi.getPWM2Velocity().getValue();
        }
    }

    
}