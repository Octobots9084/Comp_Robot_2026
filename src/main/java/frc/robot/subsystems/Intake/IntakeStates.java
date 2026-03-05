package frc.robot.subsystems.Intake;

public enum IntakeStates {
    INTAKING(75, 0.32666),
    EXTENDED(0, 0.32666),
    SAFE(0, 0),
    REVERSEINTAKING(-5, 0.32666),
    ZERO(0, 0);

    public final float rollerVoltage;
    public final double intakePosition;

    private IntakeStates(float rollerVoltage, double intakePosition) {

        this.rollerVoltage = rollerVoltage;
        this.intakePosition = intakePosition;
    }
}
