package frc.robot.subsystems.Lights;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
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
    public LightAnimations lastState = lightsCurrentState;
    public static Lights currentLightInstance;
    public static LightsIOSystem device;

    public void periodic() {
        Logger.recordOutput("lightCurrentState", this.lightsCurrentState);
        lightStateTransitions();
        applyStates();
    }

    public Lights() {
        currentLightInstance = this;
        device = new LightsIOSystem();
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
            case SHOOTHUB:
                lightsCurrentState = LightAnimations.SHOOTHUB;
                break;
            case SHOOTFERRY:
                lightsCurrentState = LightAnimations.SHOOTFERRY;
                break;
            case ZEROED:
                lightsCurrentState = LightAnimations.ZEROED;
            case DISABLED:
                lightsCurrentState = LightAnimations.DISABLED;
            break;
        }
    }

    public LightAnimations getWantedLightState() {
        return this.lightsWantedState;
    }

    public void applyStates() {
        //if (lightsWantedState != lastState)
            device.candle.setControl(lightsCurrentState.animation);
        //lastState = lightsWantedState;
    }
}
