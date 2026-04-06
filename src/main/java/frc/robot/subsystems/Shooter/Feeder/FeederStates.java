package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(15, 8),
    FERRYING(15, 8),
    OFF(0, 0),
    UNJAM(-5,-3.5),
    SPITTING(15, 3.5),
    FIXEDFIRE(15, 3.5);


    public double spindexerRPS;
    public double feederRPS;

    private FeederStates(double spindexerRPS, double feederRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
    }
}
