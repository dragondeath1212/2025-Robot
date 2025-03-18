package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToTarget extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private final PIDController m_yawController = new PIDController(0.05, 0, 0.01);
    private final PIDController m_strafeController = new PIDController(0.01, 0, 0);

    private final DoublePublisher m_yawPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/yaw")
        .publish();

    private final DoublePublisher m_horizontalOffsetPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/horizontalOffset")
        .publish();

    private final IntegerPublisher m_targetPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getIntegerTopic("vision/align/target")
        .publish();

    public AlignToTarget(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        m_yawController.reset();
        m_yawController.setSetpoint(0);

        m_strafeController.reset();
        m_strafeController.setSetpoint(0);
        m_strafeController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        var target = m_swerveSubsystem.getBestTarget();
        if (target == null) {
            return;
        }

        var cameraPixelWidth = 960; // TODO: is this configured somewhere? camera resolution appears to be 960x720
        var centerOfTarget = (target.detectedCorners.get(1).x - target.detectedCorners.get(0).x) / 2 + target.getDetectedCorners().get(0).x;
        var centerOfCamera = cameraPixelWidth / 2;
        var horizontalOffset = centerOfTarget - centerOfCamera;

        m_yawPublisher.set(target.yaw);
        m_targetPublisher.set(target.fiducialId);
        m_horizontalOffsetPublisher.set(horizontalOffset);

        var strafeVelocity = m_strafeController.calculate(horizontalOffset);
        var rotationVelocity = m_yawController.calculate(target.yaw);

        m_swerveSubsystem.drive(
            new Translation2d(0, strafeVelocity),
            rotationVelocity,
            false
        );
    }    

    @Override
    public boolean isFinished() {
        return m_strafeController.atSetpoint() && m_yawController.atSetpoint();
    }
}
