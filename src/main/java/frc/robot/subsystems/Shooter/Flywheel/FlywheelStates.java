package frc.robot.subsystems.Shooter.Flywheel;

public enum FlywheelStates {
    SAFE(0,0),
    FERRY(0,0),
    BUMP(0,0),
    HUB(0,0);

    public double FlywheelLeftRPS;
    public double FlywheelRightRPS;
    private FlywheelStates(double multiFeederRPS, double singleFeederRPS){
        this.FlywheelLeftRPS = multiFeederRPS;
        this.FlywheelRightRPS = singleFeederRPS;
    }
}
