package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;

public class Vision {
    private final PhotonCamera intakeCamera;

    public Vision(){
        intakeCamera = new PhotonCamera("FrontRight");
        intakeCamera.setDriverMode(true);

    }

    public void periodic(){

    }
}
