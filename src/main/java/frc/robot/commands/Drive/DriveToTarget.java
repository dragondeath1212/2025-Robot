package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.PoseConversion;

public class DriveToTarget extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private AprilTag m_target;
    private Command m_driveToPoseCommand;

    public DriveToTarget(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_target = m_swerveSubsystem.getNearestReef();

        if (m_target == null) {
            return;
        }

        var pose3d = m_swerveSubsystem.getTagPose(m_target);
        if (pose3d.isEmpty()) {
            return;
        }
        
        var targetPose = PoseConversion.convertPose3dToPose2d(pose3d.get())
            .transformBy(new Transform2d(Meter.of(-0.5), Meter.of(0.25), Rotation2d.kZero));

        m_driveToPoseCommand = m_swerveSubsystem.driveToPose(targetPose);

        m_driveToPoseCommand.initialize();
    }

    @Override
    public void execute() {
        m_driveToPoseCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveToPoseCommand.end(interrupted);

        m_target = null;
        m_driveToPoseCommand = null;
    }

    @Override
    public boolean isFinished() {
        if (m_driveToPoseCommand == null) {
            return true;
        }

        return m_driveToPoseCommand.isFinished();
    }
}
