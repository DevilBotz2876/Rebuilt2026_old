package frc.robot.subsystems.implementations.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.CameraInputsAutoLogged;
import frc.robot.subsystems.interfaces.Vision;
import frc.robot.subsystems.interfaces.Vision.Camera.VisionPoseMeasurement;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase implements Vision {
  List<Camera> cameras;
  List<CameraInputsAutoLogged> cameraInputs;
  AprilTagFieldLayout fieldLayout;
  Optional<VisionMeasurementConsumer> visionMeasurementConsumer;

  public VisionSubsystem(AprilTagFieldLayout layout) {
    fieldLayout = layout;
    cameras = new LinkedList<>();
    cameraInputs = new LinkedList<>();
    visionMeasurementConsumer = Optional.empty();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.size(); i++) {
      updateCamera(i);
    }
  }

  @Override
  public void updateCamera(int cameraIndex) {
    Camera camera = cameras.get(cameraIndex);
    CameraInputsAutoLogged inputs = cameraInputs.get(cameraIndex);

    Logger.processInputs("Vision/" + camera.getName(), inputs);
    camera.updateInputs(inputs);

    if (visionMeasurementConsumer.isPresent()) {
      List<VisionPoseMeasurement> validPoseMeasurements = new LinkedList<>();
      VisionPoseMeasurement[] poseMeasurements = camera.getVisionPoseMeasurements();

      // get valid measurement
      for (int i = 0; i < poseMeasurements.length; i++) {
        if (isValidPoseMeasurement(poseMeasurements[i])) {
          validPoseMeasurements.add(poseMeasurements[i]);
        }
      }

      // add the pose info to visionMeasurementConsumer
      for (int i = 0; i < validPoseMeasurements.size(); i++) {
        visionMeasurementConsumer
            .get()
            .add(
                validPoseMeasurements.get(i).robotPose,
                validPoseMeasurements.get(i).timestamp,
                VecBuilder.fill(0, 0, 0)); // Need to learn more
      }
    }
  }

  @Override
  public boolean isValidPoseMeasurement(VisionPoseMeasurement poseMeasurement) {
    // checks pose the data's confindence or ambiguity
    // need to do still
    if (poseMeasurement.targetCount > 0) {
      return true;
    }

    return false;
  }

  @Override
  public void addCamera(Camera camera) {
    cameras.add(camera);
    cameraInputs.add(new CameraInputsAutoLogged());
  }

  @Override
  public List<Camera> getCameras() {
    return cameras;
  }

  @Override
  public void setVisionMeasurementConsumer(VisionMeasurementConsumer func) {
    visionMeasurementConsumer = Optional.of(func);
  }

  @Override
  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }
}
