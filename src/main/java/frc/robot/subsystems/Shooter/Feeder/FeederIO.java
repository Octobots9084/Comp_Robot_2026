package frc.robot.subsystems.Shooter.Feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public FeederStates feederCurrentState;
    public double spindexerRPS = 0.0;
    public double verticalFeederRPS = 0.0;
    public double spindexerMotorTemp = 0.0;
    public double verticalFeederMotorTemp = 0.0;
    public double SpindexerCurrent = 0.0;
    public double verticalFeederCurrent = 0.0;

  }

  public default void updateInputs(FeederIOInputs inputs) {
  }

  public default void setFeederVelocity(FeederStates state) {
  }

  public default double getSpindexerVelocity() {
    return 0;
  }

  public default double getVerticalFeederVelocity() {
    return 0;
  }

  public default double[] getFeederVelocity() {
    double[] defaultOut = new double[2];
    return defaultOut;
  }

  public default boolean spindexerInTolerance(double tolerance) {
    return false;
  }

  public default boolean verticalFeederInTolerance(double tolerance) {
    return false;
  }

  public default boolean feederInTolerance(double tolerance) {
    return false;
  }
}