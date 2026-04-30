package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(6,8,13),//10
    FERRYING(6, 8,13),
    SPINUP(0,8,0),
    OFF(0, 0,0),
    UNJAM(-6,-6,-8),
    SPITTING(6, 8,13),
    FIXEDFIRE(6, 8,13);


    public double spindexerRPS;
    public double feederRPS;
    public double gateRPS;

    private FeederStates(double spindexerRPS, double feederRPS, double gateRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
        this.gateRPS = gateRPS;
    }
}
