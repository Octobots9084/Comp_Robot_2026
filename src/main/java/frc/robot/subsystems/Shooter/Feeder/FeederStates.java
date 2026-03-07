package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(25, -45),
    FERRYING(25, -45),
    OFF(0, 0),
    UNJAM(-15,15),
    SPITTING(2, -5);

    public double spindexerRPS;
    public double feederRPS;

    private FeederStates(double spindexerRPS, double feederRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
    }
}
