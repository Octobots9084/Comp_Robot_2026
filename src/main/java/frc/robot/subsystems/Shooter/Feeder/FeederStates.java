package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(62, 8),
    FERRYING(62, 8),
    OFF(0, 0),
    UNJAM(-30,-3.5),
    SPITTING(62, 3.5),
    FIXEDFIRE(62, 3.5);


    public double spindexerRPS;
    public double feederRPS;

    private FeederStates(double spindexerRPS, double feederRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
    }
}
