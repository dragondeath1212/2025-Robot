package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseConversion {
    public static Pose2d convertPose3dToPose2d(Pose3d pose3d) {
        // Extract the 3D translation
        Translation3d translation3d = pose3d.getTranslation();
        
        // Project the 3D translation onto the 2D plane (ignoring the z-coordinate)
        Translation2d translation2d = new Translation2d(translation3d.getX(), translation3d.getY());
        
        // Extract the 3D rotation and convert it to 2D rotation
        Rotation2d rotation2d = new Rotation2d(pose3d.getRotation().getZ())
            .rotateBy(Rotation2d.k180deg); // TODO i think the camera is wrong, so should fix the camera and remove this
        
        // Create and return the 2D pose
        return new Pose2d(translation2d, rotation2d);
    }
}