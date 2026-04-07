package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(9, 3.5),
    FERRYING(7, 9),
    OFF(0, 0),
    UNJAM(-5,-3.5),
    SPITTING(7, 3.5),
    FIXEDFIRE(7, 3.5);


    public double spindexerRPS;
    public double feederRPS;

    private FeederStates(double spindexerRPS, double feederRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
    }
}
