package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateRobot extends Command {
    private static final double MAGNITUDE = 0.25;
    private final RotationDirection m_direction;
    private final SwerveSubsystem m_swerveSubsystem;

    public RotateRobot(RotationDirection direction, SwerveSubsystem swerveSubsystem) {
        m_direction = direction;
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute() {
        var rotation = m_direction == RotationDirection.Clockwise
            ? -MAGNITUDE
            : MAGNITUDE;

        m_swerveSubsystem.drive(
            new Translation2d(0, 0),
            rotation, false
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
