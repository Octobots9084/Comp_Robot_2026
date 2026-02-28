package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double FlywheelLeftRPS = 0.0;
    public double FlywheelLeftMotorTemp = 0.0;
    public double FlywheelRightRPS = 0.0;
    public double FlywheelRightMotorTemp = 0.0;
    public double FlywheelLeftCurrent = 0.0;
    public double FlywheelRightCurrent = 0.0;

  }

  public default void updateInputs(FlywheelIOInputs inputs) {
  }

  public default void setFlywheelVelocity(FlywheelStates state) {
  }
  public default void setFlywheelVelocity(double state) {
  }

  public default double[] getFlywheelVelocity() {
    double[] defaultOut = new double[2];
    return defaultOut;
  }

  public default double getRightMotorVelocity() {
    return 0;
  }

  public default double getLeftMotorVelocity() {
    return 0;
  }

  public default boolean FlywheelInTolerance(double tolerance) {
    return false;
  }
}
