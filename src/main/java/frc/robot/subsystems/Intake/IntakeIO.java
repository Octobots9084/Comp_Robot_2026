package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

//import com.revrobotics.

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double intakePosition = 0;
        public double intakeRPS = 0;
        public double intakeExtendMotorTemp = 0;
        public double intakeMotorTemp = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs){}

    public default void setIntakeState(IntakeStates state){}

    public default double getPivotPosition(){
        return 0.0;
    }

    public default double getIntakeVelocity(){
        return 0.0;
    }

    public default boolean pivotInTolerance(double tolerance){
        return false;
    }

    public default boolean rollerInTolerance(double tolerance){
        return false;
    }

    public default boolean intakeInTolerance(double tolerance){
        return false;
    }

}
