package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSub extends SubsystemBase{

    private static final PhotonCamera CAM = new PhotonCamera(VisionConstants.CAMname);
    private static PhotonPipelineResult CAMresult = new PhotonPipelineResult();

    private static Transform3d RCtoTarget = VisionConstants.zeroTransform3d;

    public VisionSub() {
        
    }

    public static boolean hasTarget() {
        return CAMresult.hasTargets();
    }

    public static int getTargetID() {
        return CAMresult.getBestTarget().getFiducialId();
    }

    public static Transform3d getRCtoTarget() {
        RCtoTarget = hasTarget() ? 
            CAMresult.getBestTarget().getBestCameraToTarget().plus(VisionConstants.RCtoCAM)
            : VisionConstants.zeroTransform3d;
        if(getTargetID() == 4 || getTargetID() == 7) RCtoTarget = RCtoTarget.plus(VisionConstants.Id47ToSpeakerC);
        if(getTargetID() == 3 || getTargetID() == 8) RCtoTarget = RCtoTarget.plus(VisionConstants.Id38ToSpeakerC);
        return RCtoTarget;
    }

    /** @return  */
    public static double calculateShooterAngle() {
        if(getTargetID() == 3 || getTargetID() == 4 || getTargetID() == 7 || getTargetID() == 8) {
            Transform3d vector = getRCtoTarget();
            double angle = Math.atan(vector.getZ()/Math.sqrt(Math.pow(vector.getX(),2) + Math.pow(vector.getY(), 2)));
            return angle;
        } else return 0.0;
        
    }

    @Override
    public void periodic() {
        CAMresult = CAM.getLatestResult();
        Transform3d vector = getRCtoTarget();
        SmartDashboard.putBoolean("hasTarget", hasTarget());
        SmartDashboard.putNumber("TargetID", hasTarget() ? getTargetID() : -1);
        SmartDashboard.putNumber("dx", vector.getX());
        SmartDashboard.putNumber("dy", vector.getY());
        SmartDashboard.putNumber("dz", vector.getZ());
    }
}