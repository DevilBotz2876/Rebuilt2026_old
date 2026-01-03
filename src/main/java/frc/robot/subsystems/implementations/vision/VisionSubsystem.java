package frc.robot.subsystems.implementations.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.CameraInputsAutoLogged;
import frc.robot.subsystems.interfaces.Vision;
import frc.robot.subsystems.interfaces.Vision.Camera.VisionPoseMeasurement;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase implements Vision {
  private List<Camera> cameras;
  private List<CameraInputsAutoLogged> cameraInputs;
  private List<Map<Integer,List<VisionPoseMeasurement>>> cameraTagPoses = new LinkedList<>();
  private AprilTagFieldLayout fieldLayout;
  private Optional<VisionMeasurementConsumer> visionMeasurementConsumer;

  private final double timestampTolereanceInSeconds = 0.2;
  private final double validSingleTagDistanceMeter = 2.0;

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

    List<VisionPoseMeasurement> validPoseMeasurements = new LinkedList<>();
    
    for (int cameraIndex = 0; cameraIndex < cameras.size(); cameraIndex++) {
      int i = -1;
      for(VisionPoseMeasurement poseMeasurement : cameras.get(cameraIndex).getVisionPoseMeasurements()) {
        i++;
        boolean isValid = false;

        // if already valid no need to check
        if(validPoseMeasurements.contains(poseMeasurement)) {
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/vaild", true);
          continue;
        }
        // more than one tag then valid, other checks not need
        if(poseMeasurement.targetIds.length > 1) {
          validPoseMeasurements.add(poseMeasurement);
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/vaild", true);
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/reason", "MultiTag with IDs:" + Arrays.toString(poseMeasurement.targetIds));
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/matchingMeasurement", "N/A");
          continue;
        }

        // one tag seen but is close then valid
        if(poseMeasurement.robotToBestTargetDistanceInMeters > -1 && poseMeasurement.robotToBestTargetDistanceInMeters <= validSingleTagDistanceMeter) {
          validPoseMeasurements.add(poseMeasurement);
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/vaild", true);
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/reason", "Single tag with distance of :" + poseMeasurement.robotToBestTargetDistanceInMeters);
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/matchingMeasurement", "N/A");
          continue;
        }

        for(int knownTagsCameraIndex = 0; knownTagsCameraIndex < cameraTagPoses.size(); knownTagsCameraIndex++) {
          // Dont check the same camera as the poseMeasurement
          if(cameraIndex == knownTagsCameraIndex) {
            continue;
          }
          Map<Integer, List<VisionPoseMeasurement>> seenTagIdmap = cameraTagPoses.get(knownTagsCameraIndex);
          for(int aprilTagId : poseMeasurement.targetIds) {
            // if the aprilTagId is in the map (may not be needed)
            if(!seenTagIdmap.containsKey(aprilTagId)) {
              continue;
            }

            for(int seenAprilTagIdPoseMeasurementIndex = 0; seenAprilTagIdPoseMeasurementIndex < seenTagIdmap.get(aprilTagId).size(); seenAprilTagIdPoseMeasurementIndex++) {
              // if the measurements where at different times, then dont comapare
              if(Math.abs(poseMeasurement.timestamp - seenTagIdmap.get(aprilTagId).get(seenAprilTagIdPoseMeasurementIndex).timestamp) > timestampTolereanceInSeconds) {
                continue;
              }

              validPoseMeasurements.add(poseMeasurement);
              isValid = true;

              Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/vaild", true);
              Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/reason", "Same AprilTag as different camera (ID:" + aprilTagId + " )");
              Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/matchingMeasurement", "CameraName:" + cameras.get(knownTagsCameraIndex).getName() + " PoseIndex: " + seenAprilTagIdPoseMeasurementIndex);

              if(!validPoseMeasurements.contains(seenTagIdmap.get(aprilTagId).get(seenAprilTagIdPoseMeasurementIndex))) {
                validPoseMeasurements.add(seenTagIdmap.get(aprilTagId).get(seenAprilTagIdPoseMeasurementIndex));
                Logger.recordOutput("Vision/" + cameras.get(knownTagsCameraIndex).getName() + "/PoseMeasurements/" + seenAprilTagIdPoseMeasurementIndex + "/vaild", true);
                Logger.recordOutput("Vision/" + cameras.get(knownTagsCameraIndex).getName() + "/PoseMeasurements/" + seenAprilTagIdPoseMeasurementIndex + "/reason", "Same AprilTag as different camera (ID:" + aprilTagId + " )");
                Logger.recordOutput("Vision/" + cameras.get(knownTagsCameraIndex).getName() + "/PoseMeasurements/" + seenAprilTagIdPoseMeasurementIndex + "/matchingMeasurement", "CameraName:" + cameras.get(cameraIndex).getName() + " PoseIndex: " + i);
              }
              break;
            }
          }
        }
        if(!isValid) {
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/vaild", false);
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/reason", "Single Tag with no match and is greater than min valid distance");
        }
      }
    }

    for (int i = 0; i < validPoseMeasurements.size(); i++) {
      visionMeasurementConsumer
          .get()
          .add(
              validPoseMeasurements.get(i).robotPose,
              validPoseMeasurements.get(i).timestamp,
              VecBuilder.fill(0, 0, 0)); // Need to learn more
    }

    cameraTagPoses.clear();
  
  }

  @Override
  public void updateCamera(int cameraIndex) {
    Camera camera = cameras.get(cameraIndex);
    CameraInputsAutoLogged inputs = cameraInputs.get(cameraIndex);
    cameraTagPoses.add(new HashMap<>());
    Logger.processInputs("Vision/" + camera.getName(), inputs);
    camera.updateInputs(inputs);

    if (visionMeasurementConsumer.isPresent()) {
      
      VisionPoseMeasurement[] poseMeasurements = camera.getVisionPoseMeasurements();

      // get valid measurement
      for (int i = 0; i < poseMeasurements.length; i++) {

        if(poseMeasurements[i].targetIds.length == 0) {
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/vaild", false);
          Logger.recordOutput("Vision/" + cameras.get(cameraIndex).getName() + "/PoseMeasurements/" + String.valueOf(i) + "/reason", "No April Tags");
          continue;
        }
        
        for (int j = 0; j < poseMeasurements[i].targetIds.length; j++) {
          if(cameraTagPoses.get(cameraIndex).containsKey(poseMeasurements[i].targetIds[j])) {
            cameraTagPoses.get(cameraIndex).get(poseMeasurements[i].targetIds[j]).add(poseMeasurements[i]);
          } else {
            List<VisionPoseMeasurement> poseMeasurementList = new LinkedList<VisionPoseMeasurement>();
            poseMeasurementList.add(poseMeasurements[i]);
            cameraTagPoses.get(cameraIndex).put(poseMeasurements[i].targetIds[j], poseMeasurementList);
          }
        }
        Logger.recordOutput("Vision/" + camera.getName() + "/PoseMeasurements/" + String.valueOf(i) + "/timestamp", poseMeasurements[i].timestamp);
        Logger.recordOutput("Vision/" + camera.getName() + "/PoseMeasurements/" + String.valueOf(i) + "/targetIds", poseMeasurements[i].targetIds);
        Logger.recordOutput("Vision/" + camera.getName() + "/PoseMeasurements/" + String.valueOf(i) + "/robotPose", poseMeasurements[i].robotPose);
        Logger.recordOutput("Vision/" + camera.getName() + "/PoseMeasurements/" + String.valueOf(i) + "/bestTargetDistance", poseMeasurements[i].robotToBestTargetDistanceInMeters);
      }
    }

      // add the pose info to visionMeasurementConsumer
      
      
  }

  @Override
  public boolean isValidPoseMeasurement(VisionPoseMeasurement poseMeasurement) {
    // checks pose the data's confindence or ambiguity
    // need to do still
    // if (poseMeasurement.targetIds.length > 1) {
      return true;
    // }

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
