package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonCamera2;
// import org.photonvision.PhotonCamera.SimplePipelineResult;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;


public class Vision extends Subsystem {
    public final PhotonCamera camera = new PhotonCamera("MyCamera");

    
    public Vision() {
    //  PhotonPipelineResult result = camera.getLatestResult();


    //  List<PhotonTrackedTarget> targets = result.getTargets();
    //  PhotonTrackedTarget target = targets.get(0);

    //  // Get the yaw, pitch, and area from the camera.
    //  double yaw = camera.getBestTargetYaw();
    //  double pitch = camera.getBestTargetPitch();
    //  double area = camera.getBestTargetArea();

     
    //     camera.setDriverMode(true);
    //     camera.setPipelineIndex(2);

    //     // Check if the latest result has any targets.
    //     boolean hasTargets = result.hasTargets();

    //     // Get a list of currently tracked targets.
    //     // Get the pipeline latency.
    //     double latencySeconds = result.getLatencyMillis() / 1000.0;

    //     // Get information from target.
    //     double targetYaw = target.getYaw();
    //     double targetPitch = target.getPitch();
    //     double targetArea = target.getArea();
    //     double targetSkew = target.getSkew();
    //     Transform2d pose = target.getCameraToTarget();
    //     // Pose2d pose = target.getRobotRelativePose();


    //     // Get distance to target.
    //     double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
    //             Constants.Vision.kCameraHeight, Constants.Vision.kTargetHeight, 
    //             Constants.Vision.kCameraPitch, Math.toRadians(camera.getBestTargetPitch()));
    //     //Calculate a translation from the camera to the target.
    //      Translation2d translation = PhotonUtils.estimateTargetTranslation2d(
    //          distanceMeters, Rotation2d.fromDegrees(-camera.getBestTargetYaw()));



    }
    
     public double getTargetYaw() {
        //  PhotonPipelineResult result = camera.getLatestResult();
        //  List<PhotonTrackedTarget> targets = result.getTargets();
        //  if (!targets.isEmpty()) {
        //     PhotonTrackedTarget target = targets.get(0);

        //     double targetYaw = target.getYaw();
            
        //     return targetYaw;
        //  }
         return 0;
     }  

     public double getTargetPitch() {
         PhotonPipelineResult result = camera.getLatestResult();
         List<PhotonTrackedTarget> targets = result.getTargets();
         if (!targets.isEmpty()) {
            PhotonTrackedTarget target = targets.get(0);

            double targetPitch = target.getPitch();
            
            return targetPitch;
         }
         return 0;
     }  

     public boolean hasTargets() {
         PhotonPipelineResult result = camera.getLatestResult();

         boolean hasTargets = result.hasTargets();

     return hasTargets;
     }

     @Override
     protected void initDefaultCommand() {
         // TODO Auto-generated method stub

     }

} 
