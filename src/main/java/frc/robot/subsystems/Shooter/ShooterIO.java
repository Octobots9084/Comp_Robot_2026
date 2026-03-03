package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public ShooterStates ShooterCurrentState;
    public ShooterStates ShooterWantedState;
    public double turretHubAngle;
    public double hoodHubAngle;
    public boolean isAimedAtHub;
    public double flywheelCalculatorVelocity;
    public double timer;
  }

  public default void updateInputs(ShooterIOInputs inputs) {
  }
}
