package frc.robot.subsystems.Intake;

public enum IntakeStates {
    INTAKING(7, 0.34),
    EXTENDED(0, 0.34),
    SAFE(0, 0),
    REVERSEINTAKING(-5, 0.3),
    ZERO(0, 0);

    public final double rollerRPS;
    public final double intakePosition;

    private IntakeStates(double rollerRPS, double intakePosition) {

        this.rollerRPS = rollerRPS;
        this.intakePosition = intakePosition;
    }
}
