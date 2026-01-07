package frc.robot.subsystems.implementations.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.CameraInputsAutoLogged;
import frc.robot.subsystems.interfaces.Vision;
import frc.robot.subsystems.interfaces.Vision.Camera.VisionPoseMeasurement;
import java.util.ArrayList;
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
  // for each camera, a map of the seen aprilTags to an list of poseMeasurements with that april tag
  // seen
  private List<Map<Integer, List<VisionPoseMeasurement>>> cameraTagPoses = new LinkedList<>();
  private AprilTagFieldLayout fieldLayout;
  private Optional<VisionMeasurementConsumer> visionMeasurementConsumer;

  // maximum allow time between 2 poseMeasurements to still be considered vaild, need to tune
  private final double TIMESTAMP_TOLERANCE_SECONDS = 0.5;
  // maximum distance between robot and tag for poseMeasurement to be vailid to be considered
  private final double MAXIMUM_SINGLE_TAG_DISTANCE_METERS = 2.0; // need to tune to robot
  // show if pose measurement is valid, reason (and matching pose if there is one) and
  // poseMeasurement data at
  // AdvagtageKit/RealOutputs/Vision/'cameraName'/PoseMeasurements/'poseIndex'/
  private final boolean VISION_LOGGING_DEBUG = true;

  public VisionSubsystem(
      AprilTagFieldLayout layout, Optional<VisionMeasurementConsumer> visionMeasurementConsumer) {
    fieldLayout = layout;
    cameras = new LinkedList<>();
    cameraInputs = new LinkedList<>();
    this.visionMeasurementConsumer = visionMeasurementConsumer;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.size(); i++) {
      cameraTagPoses.add(new HashMap<>());
      updateCamera(i);
    }

    List<VisionPoseMeasurement> validPoseMeasurements = new LinkedList<>();

    String poseMatchDebugInfo = "N/A";
    String otherPoseMatchDebugInfo = "N/A";
    for (int cameraIndex = 0; cameraIndex < cameras.size(); cameraIndex++) {
      int i = -1;
      for (VisionPoseMeasurement poseMeasurement :
          cameras.get(cameraIndex).getVisionPoseMeasurements()) {
        i++;
        // if already valid no need to check
        if (validPoseMeasurements.contains(poseMeasurement)) {
          continue;
        }

        // more than one tag then valid, other checks not need
        if (poseMeasurement.targetIds.length >= 2) {
          validPoseMeasurements.add(poseMeasurement);
          continue;
        } else {

          // one tag seen but is close then valid
          if (poseMeasurement.robotToBestTargetDistanceInMeters != -1
              && poseMeasurement.robotToBestTargetDistanceInMeters
                  <= MAXIMUM_SINGLE_TAG_DISTANCE_METERS) {
            validPoseMeasurements.add(poseMeasurement);
            continue;
          } else {
            // check poseMeasurement from other cameras for matching tag
            boolean matchingTag = false;
            VisionPoseMeasurement matchedMeasurement = null;
            for (int knownTagsCameraIndex = 0;
                knownTagsCameraIndex < cameraTagPoses.size();
                knownTagsCameraIndex++) {
              // Dont check the same camera as the poseMeasurement
              if (cameraIndex == knownTagsCameraIndex) {
                continue;
              }
              Map<Integer, List<VisionPoseMeasurement>> seenTagIdmap =
                  cameraTagPoses.get(knownTagsCameraIndex);
              for (int aprilTagId : poseMeasurement.targetIds) {
                // if the aprilTagId is in the map
                if (!seenTagIdmap.containsKey(aprilTagId)) {
                  continue;
                }

                List<VisionPoseMeasurement> posesAtSeenTag = seenTagIdmap.get(aprilTagId);

                VisionPoseMeasurement possiblePoseMeasurementMatch =
                    posesAtSeenTag.get(posesAtSeenTag.size() - 1); // get latest

                // if the measurements where at different times, then dont comapare
                if (Math.abs(poseMeasurement.timestamp - possiblePoseMeasurementMatch.timestamp)
                    > TIMESTAMP_TOLERANCE_SECONDS) {
                  continue;
                }

                matchingTag = true;
                matchedMeasurement = possiblePoseMeasurementMatch;
                poseMatchDebugInfo =
                    "Camera: "
                        + cameras.get(knownTagsCameraIndex).getName()
                        + " MatchingTagId: "
                        + aprilTagId
                        + " timestamp: "
                        + possiblePoseMeasurementMatch.timestamp;
                otherPoseMatchDebugInfo =
                    "Camera: "
                        + cameras.get(cameraIndex).getName()
                        + " MatchingTagId: "
                        + aprilTagId
                        + " timestamp: "
                        + poseMeasurement.timestamp;
                if (matchingTag) {
                  break;
                }
              }
              if (matchingTag) {
                break;
              }
            }
            if (matchingTag) {
              validPoseMeasurements.add(poseMeasurement);
              if (VISION_LOGGING_DEBUG) {
                Logger.recordOutput(
                    "Vision/"
                        + cameras.get(cameraIndex).getName()
                        + "/PoseMeasurements/"
                        + String.valueOf(i)
                        + "/matchingMeasurementInfo",
                    poseMatchDebugInfo);
              }
              if (!validPoseMeasurements.contains(matchedMeasurement)) {
                validPoseMeasurements.add(matchedMeasurement);
                if (VISION_LOGGING_DEBUG) {
                  Logger.recordOutput(
                      "Vision/"
                          + cameras.get(cameraIndex).getName()
                          + "/PoseMeasurements/"
                          + String.valueOf(i)
                          + "/matchingMeasurementInfo",
                      otherPoseMatchDebugInfo);
                }
              }
            }
          }
        }
      }
    }

    // debug info
    if (VISION_LOGGING_DEBUG) {
      for (int cameraIndex = 0; cameraIndex < cameras.size(); cameraIndex++) {
        for (int i = 0; i < cameras.get(cameraIndex).getVisionPoseMeasurements().length; i++) {
          VisionPoseMeasurement poseMeasurement =
              cameras.get(cameraIndex).getVisionPoseMeasurements()[i];
          boolean isValid = validPoseMeasurements.contains(poseMeasurement);
          String reason = "";

          if (poseMeasurement.targetIds.length >= 2) {
            reason = "MultiTag with IDs:" + Arrays.toString(poseMeasurement.targetIds);
          } else if (poseMeasurement.robotToBestTargetDistanceInMeters != -1
              && poseMeasurement.robotToBestTargetDistanceInMeters
                  <= MAXIMUM_SINGLE_TAG_DISTANCE_METERS) {
            reason =
                "Single tag with distance of : "
                    + poseMeasurement.robotToBestTargetDistanceInMeters;
          } else if (isValid) {
            reason = "Cross Tag Check,  Same AprilTag as different camera";

          } else {
            reason =
                "Failed to have Single Tag with a match and is greater than min valid distance: "
                    + poseMeasurement.robotToBestTargetDistanceInMeters;
          }

          Logger.recordOutput(
              "Vision/"
                  + cameras.get(cameraIndex).getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/reason",
              reason);
          Logger.recordOutput(
              "Vision/"
                  + cameras.get(cameraIndex).getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/isValid",
              isValid);
        }
      }
    }

    for (int i = 0; i < validPoseMeasurements.size(); i++) {
      double distanceMeters = validPoseMeasurements.get(i).robotToBestTargetDistanceInMeters;
      visionMeasurementConsumer
          .get()
          .add(
              validPoseMeasurements.get(i).robotPose,
              validPoseMeasurements.get(i).timestamp,
              VecBuilder.fill(distanceMeters / 2, distanceMeters / 2, distanceMeters / 2));
    }

    cameraTagPoses.clear();
  }

  @Override
  public void updateCamera(int cameraIndex) {
    Camera camera = cameras.get(cameraIndex);
    CameraInputsAutoLogged inputs = cameraInputs.get(cameraIndex);
    Logger.processInputs("Vision/" + camera.getName(), inputs);
    camera.updateInputs(inputs);

    if (visionMeasurementConsumer.isPresent()) {

      VisionPoseMeasurement[] poseMeasurements = camera.getVisionPoseMeasurements();

      // add map measurments to tags in cameraTagPoses for camera
      for (int i = 0; i < poseMeasurements.length; i++) {
        for (int j = 0; j < poseMeasurements[i].targetIds.length; j++) {
          if (cameraTagPoses.get(cameraIndex).containsKey(poseMeasurements[i].targetIds[j])) {
            cameraTagPoses
                .get(cameraIndex)
                .get(poseMeasurements[i].targetIds[j])
                .add(poseMeasurements[i]);
          } else {
            List<VisionPoseMeasurement> poseMeasurementList =
                new ArrayList<VisionPoseMeasurement>();
            poseMeasurementList.add(poseMeasurements[i]);
            cameraTagPoses
                .get(cameraIndex)
                .put(poseMeasurements[i].targetIds[j], poseMeasurementList);
          }
        }
        if (VISION_LOGGING_DEBUG) {
          Logger.recordOutput(
              "Vision/"
                  + camera.getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/timestamp",
              poseMeasurements[i].timestamp);

          Logger.recordOutput(
              "Vision/"
                  + camera.getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/targetIds",
              poseMeasurements[i].targetIds);
          Logger.recordOutput(
              "Vision/"
                  + camera.getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/robotPose",
              poseMeasurements[i].robotPose);
          Logger.recordOutput(
              "Vision/"
                  + camera.getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/bestTargetDistance",
              poseMeasurements[i].robotToBestTargetDistanceInMeters);
        }
      }
    }
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
  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }
}
