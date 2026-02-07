package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double climbPosition = 0.0;
        public double deployPosition = 0.0;
        public double deployMotorTemperature = 0.0;
        public double climbMotorControlledTemperature = 0.0;
        public double climbMotorFollowerTemperature = 0.0;
    }

    public default void updateInputs(ClimbIOInputs inputs){}

    public default void setClimbState(ClimbStates states){}

    public default double getClimbPosition() {
        return 0;
    }

    public default double getDeployPosition() {
        return 0;
    }

    public default boolean climbInTolerance(double climbTolerance){
        return false;
    }
    public default void setRotateVoltage(double voltage){
        
    }
    public default boolean isZeroingSwitchPressed(){
        return false;
    }
}
