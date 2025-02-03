package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ArmState {
    @SuppressWarnings("unused")
    private Angle m_jointPosition;
    @SuppressWarnings("unused")
    private AngularVelocity m_jointVelocity;

    public ArmState(Angle jointPosition, AngularVelocity jointVelocity) {
        m_jointPosition = jointPosition;
        m_jointVelocity = jointVelocity;
    }

    public void setArmState(Angle jointPosition, AngularVelocity jointVelocity) {
        m_jointPosition = jointPosition;
        m_jointVelocity = jointVelocity;
    }
}
