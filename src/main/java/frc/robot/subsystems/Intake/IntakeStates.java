package frc.robot.subsystems.Intake;

public enum IntakeStates {
    INTAKING(75, 0),
    EXTENDED(0, 0),
    SAFE(0, 0),
    REVERSEINTAKING(-25, 0),
    ZERO(0,0);
    public final float rollerRPS;
    public final float intakePosition;
    
    private IntakeStates(float rollerRPS, float intakePosition){
        
        this.rollerRPS = rollerRPS;
        this.intakePosition = intakePosition;
    }
}
