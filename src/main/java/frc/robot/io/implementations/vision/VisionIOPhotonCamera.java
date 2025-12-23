package frc.robot.io.implementations.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.io.interfaces.VisionIO;

public class VisionIOPhotonCamera implements VisionIO {
    private final PhotonCamera camera;

    public VisionIOPhotonCamera(PhotonCamera camera) {
        this.camera = camera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.isConnected = camera.isConnected();
        //inputs.fps = camera.getAllUnreadResults().get(0).get
    }   

    @Override
    public String getName() {
        return "UNKNOWN_CAMERA";
    }
}