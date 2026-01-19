package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public interface FlywheelIO{
    @AutoLog
    public static class FlywheelIOInputs {
      public double FlywheelRPS = 0.0;
      public double LeftMotorTemp = 0.0;
      public double RightMotorTemp = 0.0;
  }
  public default void updateInputs(FlywheelIOInputs inputs){}

  public default void setFlyWheelVelocity(double RPS){}

  public default AngularVelocity getFlyWheelVelocity(){
    return Units.RadiansPerSecond.of(0);
  }

  public default boolean flywheelInTolerance(double tolerance){
    return false;
  }

  public default void setTopRollerVelocity(double RPS){}

  public default AngularVelocity getTopRollerVelocity(){
    return Units.RadiansPerSecond.of(0);
  }

  public default boolean topRollerInTolerance(double tolerance){
    return false;
  }
}