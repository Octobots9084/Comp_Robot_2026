package frc.robot.subsystems.Vision;

public class ShooterAngle {
    public double turretRotation;
    public double hoodRotation;
    public double turretFlywheelSpeed;

    public ShooterAngle(double turretRotation, double hoodRotation, double turretFlywheelSpeed) {
        this.turretRotation = turretRotation;
        this.hoodRotation = hoodRotation;
        this.turretFlywheelSpeed = turretFlywheelSpeed;
    }
}