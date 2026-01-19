package frc.robot.subsystems.Shooter.Flywheel;

public enum FlywheelStates {
    SCORING(0,0),
    FERRYING(0,0),
    OFF(0,0);

    public double flywheelRPS;
    public double topRollerRPS;
    private FlywheelStates(double RPS, double topRollerRPS){
        flywheelRPS = RPS;
        this.topRollerRPS = topRollerRPS;
    }
}
