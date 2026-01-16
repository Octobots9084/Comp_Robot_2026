package frc.robot.subsystems.Intake;

public enum IntakeStates {
        
    INTAKING(1, 1),
    EXTENDED(0, 1),
    SAFE(0, 0),
    REVERSEINTAKING(-1, 1);

    public final float rollerVoltage;
    public final float intakePosition;
    
    private IntakeStates(float rollerVoltage, float intakePosition){
        this.rollerVoltage = rollerVoltage;
        this.intakePosition = intakePosition;
    }
}
//67
