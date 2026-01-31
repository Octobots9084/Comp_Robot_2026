package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(0,0),
    FERRYING(0,0),
    OFF(0,0);

    public double multiFeederRPS;
    public double singleFeederRPS;
    private FeederStates(double multiFeederRPS, double singleFeederRPS){
        this.multiFeederRPS = multiFeederRPS;
        this.singleFeederRPS = singleFeederRPS;
    }
}
