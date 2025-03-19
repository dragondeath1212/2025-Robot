package frc.robot.commands.Drive;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class AlignToTarget extends Command {
    private final CommandXboxController m_controller;
    private final SwerveSubsystem m_swerveSubsystem;
    private final PIDController m_rotationController = new PIDController(0.05, 0.005, 0);
    private final PIDController m_strafeController = new PIDController(0.005, 0, 0);
    private final PIDController m_rangeController = new PIDController(2, 0, 0);

    private final DoublePublisher m_rotationOffsetPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/rotationOffset")
        .publish();

    private final DoublePublisher m_horizontalOffsetPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/horizontalOffset")
        .publish();

    private final DoublePublisher m_rangePublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/range")
        .publish();

    private final IntegerPublisher m_targetPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getIntegerTopic("vision/align/target")
        .publish();

    public AlignToTarget(CommandXboxController controller, SwerveSubsystem swerveSubsystem) {
        m_controller = controller;
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_rotationController.reset();
        m_rotationController.setSetpoint(0);
        m_rotationController.setTolerance(0.5);

        m_strafeController.reset();
        m_strafeController.setSetpoint(0);
        m_strafeController.setTolerance(0.5);

        m_rangeController.reset();
        m_rangeController.setSetpoint(
            Units.inchesToMeters(12) // note that this is the distance from the camera to the target, not the front bumper
        );
        m_rangeController.setTolerance(0.02);
    }

    @Override
    public void execute() {
        var target = m_swerveSubsystem.getBestReefTarget();
        if (target == null) {
            return;
        }

        if (!m_strafeController.atSetpoint() || !m_rotationController.atSetpoint() || !m_rangeController.atSetpoint()) {
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 1);
        } else {
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        }

        var targetHeight = m_swerveSubsystem
            .getNearestReefPosition()
            .pose
            .getZ();

        var cameraPitch = Vision.Cameras.CENTER_CAM.robotToCamTransform
            .getRotation()
            .getY();

        var cameraHeight = Vision.Cameras.CENTER_CAM.robotToCamTransform
            .getTranslation()
            .getZ();

        var centerOfTarget = (target.detectedCorners.get(1).x - target.detectedCorners.get(0).x) / 2 + target.getDetectedCorners().get(0).x;
        var centerOfCamera = VisionConstants.cameraWidth / 2;
        var horizontalOffset = centerOfTarget - centerOfCamera;

        var transform = target.getBestCameraToTarget();
        var rotationOffset = -transform.getRotation()
            .toRotation2d()
            .rotateBy(Rotation2d.k180deg)
            .getDegrees();

        var range = PhotonUtils.calculateDistanceToTargetMeters(
            cameraHeight, // Measured with a tape measure, or in CAD.
            targetHeight,
            -cameraPitch,
            Units.degreesToRadians(target.getPitch()));

        m_targetPublisher.set(target.fiducialId);
        m_rotationOffsetPublisher.set(rotationOffset);
        m_horizontalOffsetPublisher.set(horizontalOffset);
        m_rangePublisher.set(range);

        var strafeVelocity = m_strafeController.calculate(horizontalOffset);
        var rotationVelocity = m_rotationController.calculate(rotationOffset);
        var rangeVelocity = m_rangeController.calculate(range);

        m_swerveSubsystem.drive(
            new Translation2d(-rangeVelocity, strafeVelocity),
            rotationVelocity,
            false
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return m_strafeController.atSetpoint() &&
            m_rotationController.atSetpoint() &&
            m_rangeController.atSetpoint();
    }
}
