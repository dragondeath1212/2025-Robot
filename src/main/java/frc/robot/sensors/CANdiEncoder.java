package frc.robot.sensors;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.configs.PWM2Configs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;


import frc.robot.Constants.ArmConstants;

public class CANdiEncoder {
    private final CANdi m_CANdi;
    private final int m_signalId;
    private Angle m_position;
    private AngularVelocity m_velocity;

    public CANdiEncoder(CANdi m_CANdi, int signalId) {
        this.m_CANdi = m_CANdi;
        this.m_signalId = signalId;
    }

    public Angle getPosition() {
        if (m_signalId == 1) {
            return m_CANdi.getPWM1Position().getValue();
        }
        else {
            return m_CANdi.getPWM2Position().getValue();
        }
    }

    public AngularVelocity getVelocity() {
        if (m_signalId == 1) {
            return m_CANdi.getPWM1Velocity().getValue();
        }
        else {
            return m_CANdi.getPWM2Velocity().getValue();
        }
    }

    
}