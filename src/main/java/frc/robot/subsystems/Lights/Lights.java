package frc.robot.subsystems.Lights;

public class Lights {
/**
   * The current state of the lights, which determines the color of the lights
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Lights.LightAnimations#DEFAULT DEFAULT}
   * @param LightAnimations The light states contain color and strobe settings - {@link frc.robot.subsystems.Lights.LightAnimations LightAnimations}
   */
    public LightAnimations lightsCurrentState = LightAnimations.DEFAULT;
  /**
   * The wanted state of the lights, which the subsystem attempts to set the {@link #lightsCurrentState} to
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Lights.LightAnimations#DEFAULT DEFAULT}
   * @param LightAnimations The light states contain color and strobe settings - {@link frc.robot.subsystems.Lights.LightAnimations LightAnimations}
   *
   */
    public LightAnimations lightsWantedState = LightAnimations.DEFAULT;
    public static Lights currentLightInstance;

    public void periodic() {
        lightStateTransitions();
    }

    public Lights() {
        currentLightInstance = this;
    }

    public static Lights getLightInstance() {
        if (currentLightInstance == null) {
            setLightInstance(new Lights());
        }
        return currentLightInstance;
    }

    public static void setLightInstance(Lights instance) {
        currentLightInstance = instance;
    }

    public void lightStateTransitions() {
        switch (lightsWantedState) {
            case DEFAULT:
                lightsCurrentState = LightAnimations.DEFAULT;
                break;
            case INTAKING:
                lightsCurrentState = LightAnimations.INTAKING;
                break;
            case REVERSEINTAKING:
                lightsCurrentState = LightAnimations.REVERSEINTAKING;
                break;
            case CANTSHOOT:
                lightsCurrentState = LightAnimations.CANTSHOOT;
                break;
            case SHOOTREADYCONTINIOUS:
                lightsCurrentState = LightAnimations.SHOOTREADYCONTINIOUS;
                break;
            case SHOOTREADYMANUAL:
                lightsCurrentState = LightAnimations.SHOOTREADYMANUAL;
                break;
        }
    }

    public LightAnimations getWantedLightState() {
        return this.lightsWantedState;
    }

}
