package frc.robot.io.interfaces;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    @AutoLog
    public class VisionIOInputs {
        public boolean isConnected = false;
        public double fps;
        public int[] targetIds = new int[0];
        public Pose2d cameraPose;
        public PoseEsitmatorData[] poseEsitmatorData;
    }

    public static class PoseEsitmatorData {
        public double timestamp;
        public Pose2d RobotPose;
        public int targetCount;
        // somekind of distance to very it is good to use
    }

    public void updateInputs(VisionIOInputs inputs);

    public String getName();
}
