package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.VisionIOSystem.EstimateConsumer;

//implement directional 45deg rot lock for intake

public class Vision {
    public static enum VisionStates {
        BUMPING,
        CLIMB,
        SHOOTINGINHUB,
        FERRYING
    }

    private final VisionIO io;
    private VisionStates visionState = VisionStates.SHOOTINGINHUB;
    private VisionStates visionWantedState = VisionStates.SHOOTINGINHUB;

    public Vision(EstimateConsumer estConsumer){
        io = new VisionIOSystem(estConsumer);
        
    }

    public void periodic(){
        ApplyStates();
        handleStateTransitions();
    }

    public void ApplyStates(){
        switch (visionState) {
            case SHOOTINGINHUB:

                break;
            case BUMPING:

                break;
            case FERRYING:

                break;
            default:
                break;
        }
    }

    public void handleStateTransitions(){
        switch (visionWantedState) {
            case SHOOTINGINHUB:
                visionState = VisionStates.SHOOTINGINHUB;
                break;
            case BUMPING:
                visionState = VisionStates.BUMPING;
                break;
            case FERRYING:
                visionState = VisionStates.FERRYING;
                break;
            default:
                break;
        }
    }
}
