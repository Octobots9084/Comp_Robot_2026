package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public ShooterStates ShooterCurrentState;
  }

  public default void updateInputs(ShooterIOInputs inputs) {
  }
}
