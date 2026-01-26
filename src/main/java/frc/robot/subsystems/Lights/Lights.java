package frc.robot.subsystems.Lights;

public class Lights {
     public static class LightsIOInputs {
        LightAnimations animationState;
    }

    public default void setAnimation(LightAnimations animations) {}

    public default void setAnimation(LightAnimations[] animations) {}

}
