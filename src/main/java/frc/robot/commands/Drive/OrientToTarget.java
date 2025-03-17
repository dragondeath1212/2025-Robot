package frc.robot.commands.Drive;

import org.photonvision.PhotonUtils;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class OrientToTarget extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private AprilTag m_target;

    public OrientToTarget(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_target = m_swerveSubsystem.getNearestReef();

        if (m_target == null) {
            return;
        }
    }

    @Override
    public void execute() {
        var trackedTarget = m_swerveSubsystem.getTrackedTarget(m_target.ID);
        var targetYaw = trackedTarget.getYaw();

        // Calculate how much to turn each cycle to align with the target
        var turnGain = 0.01; // scale to tune how quickly the robot turns
        var maxTurnSpeed = Math.PI;
        var turn = -targetYaw * turnGain * maxTurnSpeed;
        
        var idealRange = 1;
        var strafeGain = 0.5; // scale to tune how quickly the robot moves
        var maxLinearSpeed = 4; // m/s
        var targetRange = PhotonUtils.calculateDistanceToTargetMeters(
            0.5, // Measured with a tape measure, or in CAD.
            1.435, // From 2024 game manual for ID 7
            Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
            Units.degreesToRadians(trackedTarget.getPitch()));

        var forward = (idealRange - targetRange) * strafeGain * maxLinearSpeed;
        
        
        var targetPose = m_swerveSubsystem.getTagPose(m_target);
        var strafe = 0;  
        
        // TODO figure out how to position the bot using the april tag for alignment
        // there is an aimAtTarget method that might do most of this...
        // if (targetPose.isPresent()) {
        //     var robotPose = m_swerveSubsystem.getPose();

        //     strafe = targetPose.get().
        // }

        // var targetPose2d = PoseConversion.convertPose3dToPose2d(targetPose.get());
        // var robotPose = m_swerveSubsystem.getPose();
        // var 


        // m_swerveSubsystem.drive(
        //   new Translation2d()  
        // );



        // m_driveToPoseCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_target = null;
    }

    @Override
    public boolean isFinished() {
        return true; // TODO should validate position is within some margin of error
    }
}
