package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(10, 3.5),
    FERRYING(10, 9),
    OFF(0, 0),
    UNJAM(-4,-3.5),
    SPITTING(9, 3.5),
    FIXEDFIRE(9, 3.5);


    public double spindexerRPS;
    public double feederRPS;

    private FeederStates(double spindexerRPS, double feederRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
    }
}
