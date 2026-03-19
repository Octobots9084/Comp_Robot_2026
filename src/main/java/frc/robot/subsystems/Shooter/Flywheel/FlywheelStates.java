package frc.robot.subsystems.Shooter.Flywheel;

public enum FlywheelStates {
    SAFE(0),
    FERRY(0),
    BUMP(0),
    SPIT(8), // todo: find out what this should be
    HUB(10),
    SPITTOCONTAINER(3),
    FIXEDFIRE(10)
    ;

    public double FlywheelRightRPS;

    private FlywheelStates(double flywheelRPS) {
        this.FlywheelRightRPS = flywheelRPS;
    }
}
