package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;

//implement directional 45deg rot lock

public class Vision {

    
    private final PhotonCamera intakeCamera;

    public Vision(){
        intakeCamera = new PhotonCamera("IntakeCam");
        intakeCamera.setDriverMode(true);
        CameraServer.startAutomaticCapture("IntakeCam", 0);
    }

    public void periodic(){

    }
}
