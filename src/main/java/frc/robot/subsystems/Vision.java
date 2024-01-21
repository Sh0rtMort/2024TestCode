package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    private PhotonCamera camera = new PhotonCamera("2531photonvision");


    //camera position mounted constants
    private final double cameraHeightMeters = 0; //sample Number
    private final double targetHeightMeters = 1.222375; //real height of april tags
    private final double cameraPitchRadians = Units.degreesToRadians(cameraHeightMeters); //sample degree to radians


    public Vision() {

    }

    public double getPitchOfShootingTarget() {
        // targetHeightMeters = Units.feetToMeters(10.875);
        var result = camera.getLatestResult();
        var bestTarget = result.getBestTarget();

        if (result.hasTargets()) {
            return Math.atan(10.875 / getDistanceToTarget() - bestTarget.getPitch());
        } 
        //TODO: find the radius of the whole shooter in its entirety, for now its 1 foot
        //encoder ticks Radius(ft) = 76.595, THIS IS SO WRONG LMAOO
        // return nothing because there is no target
        return 0;
    }

    public double getDistanceToTarget() {
        // targetHeightMeters = 1.222375;
        if (camera.getLatestResult().hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeightMeters, 
                targetHeightMeters, 
                cameraPitchRadians, 
                Units.degreesToRadians(camera.getLatestResult().getBestTarget().getPitch()));
        }
        // return nothing because there is no target
        return 0;
    }

    // public Translation2d translationToTarget() {
    //     return PhotonUtils.estimateCameraToTargetTranslation(getDistanceToTarget(), getTargetYaw());
    // }

    // public Rotation2d getTargetYaw() {
    //     var result = camera.getLatestResult();
    //     var bestTarget = result.getBestTarget();

    //     if (result.hasTargets()) {
    //         return PhotonUtils.getYawToPose(, null);
    //     }
    //     // return nothing because there is no target
    //     return null;
    // }

    // public Pose3d getRobotPose() {
    //     var result = camera.getLatestResult();
    //     var target = result.getBestTarget();
    //     if (result.hasTargets()) {
    //     return PhotonUtils.estimateFieldToRobotAprilTag(
    //         target.getBestCameraToTarget(), 
    //         aprilTagFieldLayout.getTagPose(target.getFiducialId()), 
    //         cameraToRobot
    //     );
    //     }
    // }


}
