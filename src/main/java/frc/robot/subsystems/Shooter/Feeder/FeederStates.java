package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(0,0),
    FERRYING(0,0),
    OFF(0,0),
    SPITTING(10,-7);

    public double spindexerRPS;
    public double feederRPS;
    private FeederStates(double spindexerRPS, double feederRPS){
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
    }
}
