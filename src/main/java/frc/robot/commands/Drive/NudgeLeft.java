package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class NudgeLeft extends Command {
    private final SwerveSubsystem m_swerveSubsystem;

    public NudgeLeft(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute() {
        m_swerveSubsystem.drive(
            new Translation2d(0, 1),
            0, false
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
