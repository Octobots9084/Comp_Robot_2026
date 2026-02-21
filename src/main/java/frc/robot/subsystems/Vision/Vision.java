package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public Vision(EstimateConsumer estConsumer) {
        io = new VisionIOSystem(estConsumer);
        SmartDashboard.putNumber("testerVx", 0);
        SmartDashboard.putNumber("testerVy", 0);
        SmartDashboard.putNumber("testerPfx", 3);
        SmartDashboard.putNumber("testerPfy", 4);
        SmartDashboard.putNumber("testerflywheelSpeed", 10);
    }

    public void periodic() {
        io.periodic();

        // double vx = SmartDashboard.getNumber("testerVx", 0);
        // double vy = SmartDashboard.getNumber("testerVy", 0);
        // double pfx = SmartDashboard.getNumber("testerPfx", 3);
        // double pfy = SmartDashboard.getNumber("testerPfy", 4);
        // double s = SmartDashboard.getNumber("testerflywheelSpeed", 10);

        // ShooterAngle testShooterAngle =
        // ShooterAngleCalculator.getShooterAngleToHub(vx, vy, pfx, pfy, s);

        // if (testShooterAngle != null){
        // SmartDashboard.putNumber("testerHoodAngle", testShooterAngle.hoodRotation);
        // SmartDashboard.putNumber("testerTurretAngle",
        // testShooterAngle.turretRotation);
        // }

        ApplyStates();
        handleStateTransitions();
    }

    public void ApplyStates() {
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

    public void handleStateTransitions() {
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
