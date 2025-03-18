package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateClockwise extends Command {
    private final SwerveSubsystem m_swerveSubsystem;

    public RotateClockwise(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute() {
        m_swerveSubsystem.drive(
            new Translation2d(0, 0),
            -1, false
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
