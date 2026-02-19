package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

//import com.revrobotics.

public interface IntakeIO {
@AutoLog
public static class IntakeIOInputs {
    public double rollerRPS = 0.0;
    public double intakePosition = 0.0;
    public double rollerTemp = 0.0;
    public double pivotTemp = 0.0;
    public double rollerCurrent = 0.0;
    public double pivotCurrent = 0.0;
}
public default void updateInputs(IntakeIOInputs inputs){}

public default void setIntakeState(IntakeStates states){}

public default double getRollerRPS(){
    return 0;
}

public default double getIntakePosition(){
    return 0;
}
public default boolean isZeroingSwitchPressed(){//GAS_D
         return false;
   }
}

