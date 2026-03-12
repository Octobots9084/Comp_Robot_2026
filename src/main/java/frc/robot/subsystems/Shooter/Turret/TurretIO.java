package frc.robot.subsystems.Shooter.Turret;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double hoodPosition;
        // public double hoodCurrent;
        public double turretPosition;
        // public double hoodMotorTemp;
        // public double turretMotorTemp;
        public double turretRequest;
        public double hoodRequest;
        // public double turretVoltage;
        // public double turretCurrent;
        public double turretPositionErr;
        public boolean turretLimitSwitch;
    }

    public default void updateInputs(TurretIOInputs inputs) {
    }

    public default boolean getAimedToShoot(){
        return false;
    }

    public default void setTurretPosition(double turretAngle) {
    }

    public default void zeroTurretPosition() {
    }

    public default void setHoodPosition(double hoodAngle) {
    }

    public default double getHoodPosition() {
        return 0.0;
    }
    public default boolean getMagnetBreakValue(){
            return false;
    }

    public default double getTurretPosition() {
        return 0.0;
    }

    public default boolean hoodInTolerance(double tolerance) {
        return false;
    }

    public default boolean turretInTolerance(double tolerance) {
        return false;
    }


    public default boolean turretZeroed() {
        return false;
    }

    public default boolean hoodZeroed() {
        return false;
    }

    public default void zeroHoodMotor(){}
}
