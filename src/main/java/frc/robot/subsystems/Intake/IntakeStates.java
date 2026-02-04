package frc.robot.subsystems.Intake;

public enum IntakeStates {
        
    INTAKING(5, 0),
    EXTENDED(0, 0),
    SAFE(0, 0),
    REVERSEINTAKING(0, 0);

    public final float rollerRPS;
    public final float intakePosition;
    
    private IntakeStates(float rollerRPS, float intakePosition){
        this.rollerRPS = rollerRPS;
        this.intakePosition = intakePosition;
    }
}
