package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(62, 3.5),
    FERRYING(25, 3.5),
    OFF(0, 0),
    UNJAM(-15,3.5),
    SPITTING(15, 3.5),
    FIXEDFIRE(25, 3.5);


    public double spindexerRPS;
    public double feederRPS;

    private FeederStates(double spindexerRPS, double feederRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
    }
}
