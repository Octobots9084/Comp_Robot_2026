package frc.robot.subsystems.Shooter.Flywheel;

public enum FlywheelStates {
    SAFE(0),
    FERRY(0),
    BUMP(0),
    HUB(0);

    public double FlywheelRightRPS;
    private FlywheelStates(double flywheelRPS){
        this.FlywheelRightRPS = flywheelRPS;
    }
}
