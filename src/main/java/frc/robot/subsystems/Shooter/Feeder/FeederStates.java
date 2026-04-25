package frc.robot.subsystems.Shooter.Feeder;

public enum FeederStates {
    SCORING(6,8,13),//10
    FERRYING(11, 13,12),
    SPINUP(0,13,0),
    OFF(0, 0,0),
    UNJAM(-4,-3.5,-4),
    SPITTING(9, 3.5,3),
    FIXEDFIRE(9, 3.5,3);


    public double spindexerRPS;
    public double feederRPS;
    public double gateRPS;

    private FeederStates(double spindexerRPS, double feederRPS, double gateRPS) {
        this.spindexerRPS = spindexerRPS;
        this.feederRPS = feederRPS;
        this.gateRPS = gateRPS;
    }
}
