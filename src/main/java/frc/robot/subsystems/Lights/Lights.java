package frc.robot.subsystems.Lights;

public class Lights {
    public LightAnimations lightsCurrentState = LightAnimations.DEFAULT;
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
