package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.io.interfaces.VisionIO;
import frc.robot.io.interfaces.VisionIO.PoseEsitmatorData;

public interface Vision {
    
    
    public Pose2d getBestTargetPose(String name);
    private void updateIO(VisionIO io) {
        // updateinputs
        // checkPoseData
        // addVisionMeasurement(validPoseStuff)
    };
    private boolean checkPoseData(PoseEsitmatorData poseData) {
        // more than one tag
        // tags are within the field
        // ambiguity > 0.2
        return true;
    }
}