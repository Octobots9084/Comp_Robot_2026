package frc.robot.subsystems.Shooter.Flywheel;

public enum FlywheelStates {
    SAFE(0),
    FERRY(0),
    BUMP(0),
    SPIT(0), // todo: find out what this should be
    HUB(44);

    public double FlywheelRightRPS;

    private FlywheelStates(double flywheelRPS) {
        this.FlywheelRightRPS = flywheelRPS;
    }
}
