package frc.robot.subsystems.Shooter.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public double hoodPosition;
        public double turretPosition;
        public double hoodMotorTemp;
        public double turretMotorTemp;
        public double turretRequest;
        public double hoodRequest;

    }

    public default void updateInputs(TurretIOInputs inputs){}

    public default void setTurretPosition(double turretAngle){}

    public default void setHoodPosition(double hoodAngle){}

    public default double getHoodPosition(){ return 0.0;}

    public default double getTurretPosition(){ return 0.0;}

    public default boolean hoodInTolerance(double tolerance){ return false;}

    public default boolean turretInTolerance(double tolerance){ return false;}
}
