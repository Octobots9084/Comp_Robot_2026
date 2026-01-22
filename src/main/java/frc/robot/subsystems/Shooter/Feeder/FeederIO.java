package frc.robot.subsystems.Shooter.Feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO{
    @AutoLog 
    public static class FeederIOInputs {
      public double MultiFeederRPS = 0.0;
      public double SingleFeederRPS = 0.0;
      public double MultiFeederMotorTemp = 0.0;
      public double SingleFeederMotorTemp = 0.0;
    }
  public default void updateInputs(FeederIOInputs inputs){}

  public default void setFeederVelocity(double mulitFeederRPS, double singleFeederRPS){}

  public default double getMultiFeederVelocity(){
    return 0;
  }

  public default double getSingleFeederVelocity(){
    return 0;
  }

  public default double[] getFeederVelocity(){
    double[] defaultOut = new double[2];
    return defaultOut;
  }

  public default boolean multiFeederInTolerance(double tolerance){
    return false;
  }

  public default boolean singleFeederInTolerance(double tolerance){
    return false;
  }

  public default boolean feederInTolerance(double tolerance){
    return false;
  }
}