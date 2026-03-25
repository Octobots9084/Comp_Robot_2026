package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(50, 5),
    FERRYING(25, 5),
    OFF(0, 0),
    UNJAM(-15,15),
    SPITTING(15, 5),
    FIXEDFIRE(25, 5);

    public double spindexerRPS;
    public double feederRPS;

    private FeederStates(double spindexerRPS, double feederRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
    }
}
