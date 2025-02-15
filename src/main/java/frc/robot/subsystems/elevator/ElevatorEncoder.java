package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.configs.PWM2Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ElevatorEncoder {
    private final CANdi m_elevatorCANdi;
    private final int m_signalId;
    private Angle m_position;
    private AngularVelocity m_velocity;
    private final PWM1Configs m_PWM1Config = new PWM1Configs();
    private final PWM2Configs m_PWM2Config = new PWM2Configs();
    private CANdiConfiguration candiConfig = new CANdiConfiguration();

    public ElevatorEncoder(CANdi m_elevatorCANdi, int signalId) {
        this.m_elevatorCANdi = m_elevatorCANdi;
        this.m_signalId = signalId;

    }

    public Angle getPosition() {
        if (m_signalId == 1) {
            return m_elevatorCANdi.getPWM1Position().getValue();
        }
        else {
            return m_elevatorCANdi.getPWM2Position().getValue();
        }
    }

    public AngularVelocity getVelocity() {
        if (m_signalId == 1) {
            return m_elevatorCANdi.getPWM1Velocity().getValue();
        }
        else {
            return m_elevatorCANdi.getPWM2Velocity().getValue();
        }
    }
}
