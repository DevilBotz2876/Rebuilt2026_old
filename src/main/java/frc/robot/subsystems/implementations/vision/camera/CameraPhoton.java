package frc.robot.subsystems.implementations.vision.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.interfaces.Vision.Camera;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraPhoton implements Camera {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private final String name;
  private final AprilTagFieldLayout tagLayout;
  private VisionPoseMeasurement[] poseMeasurements;

  public CameraPhoton(String name, Transform3d robotToCamera, AprilTagFieldLayout tagLayout) {
    this.name = name;
    camera = new PhotonCamera(this.name);
    this.robotToCamera = robotToCamera;
    this.tagLayout = tagLayout;
  }

  @Override
  public void updateInputs(CameraInputs inputs) {
    inputs.isConnected = camera.isConnected();
    // inputs.fps = unsure right now

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    poseMeasurements = new VisionPoseMeasurement[results.size()];

    for (int i = 0; i < results.size(); i++) {
      PhotonPipelineResult result = results.get(i);
      VisionPoseMeasurement measurement = new VisionPoseMeasurement();

      if (!result.hasTargets()) {
        // no ta
        measurement.targetCount = 0;
      } else if (result.multitagResult.isPresent()) {
        MultiTargetPNPResult multitagResult = result.multitagResult.get();
        Pose2d cameraPose =
            new Pose3d(Translation3d.kZero, Rotation3d.kZero)
                .plus(multitagResult.estimatedPose.best)
                .toPose2d();
        inputs.cameraPose = cameraPose;
        inputs.targetIds = new int[multitagResult.fiducialIDsUsed.size()];
        for (int j = 0; j < inputs.targetIds.length; j++) {
          inputs.targetIds[j] = multitagResult.fiducialIDsUsed.get(j).intValue();
        }

        measurement.robotPose =
            cameraPose.transformBy(
                (new Transform2d(
                        robotToCamera.getTranslation().toTranslation2d(),
                        robotToCamera.getRotation().toRotation2d()))
                    .inverse());
        measurement.targetCount = multitagResult.fiducialIDsUsed.size();
        measurement.ambiguity = multitagResult.estimatedPose.ambiguity;
        measurement.timestamp = result.getTimestampSeconds();
      } else {
        // one tag
        // use location on field to determine
      }
      poseMeasurements[i] = measurement;
    }
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  @Override
  public VisionPoseMeasurement[] getVisionPoseMeasurements() {
    return poseMeasurements;
  }

  public PhotonCamera getPhotonCamera() {
    return camera;
  }
}
