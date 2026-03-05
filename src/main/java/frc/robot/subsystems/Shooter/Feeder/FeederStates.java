package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(6, -4),
    FERRYING(6, -4),
    OFF(0, 0),
    AGITATION(2,0),
    SPITTING(5, -3);

    public double spindexerRPS;
    public double feederRPS;

    private FeederStates(double spindexerRPS, double feederRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
    }
}
