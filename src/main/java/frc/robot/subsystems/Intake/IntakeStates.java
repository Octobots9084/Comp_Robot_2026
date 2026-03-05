package frc.robot.subsystems.Intake;

public enum IntakeStates {
    INTAKING(75, 0.32666),
    EXTENDED(0, 0.32666),
    SAFE(0, 0),
    REVERSEINTAKING(-25, 0.32666),
    ZERO(0, 0);

    public final float rollerRPS;
    public final double intakePosition;

    private IntakeStates(float rollerRPS, double intakePosition) {

        this.rollerRPS = rollerRPS;
        this.intakePosition = intakePosition;
    }
}
