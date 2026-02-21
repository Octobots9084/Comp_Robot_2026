package frc.robot.subsystems.Lights;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.CANdle;

public interface LightsIO {
   @AutoLog
   public static class LightsIOInputs {
      LightAnimations animationState;
   }

   public default void setAnimation(LightAnimations animations) {
   }

   public default void setAnimation(LightAnimations[] animations) {
   }

   public default void updateInputs(LightsIOInputs inputs) {
   }

   public default void playAnimation() {
   }

   public default CANdle getcandle() {
      return null;
   }
}
