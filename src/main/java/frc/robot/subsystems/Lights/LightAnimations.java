package frc.robot.subsystems.Lights;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;


public enum LightAnimations {

    DEFAULT(new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 255, 0, 0)), 0.1), // Yellow
    CANTSHOOT(new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 50, 0, 0)), 0.1), // Orange
    SHOOTFERRY(new StrobeAnimation(0, 65).withColor(new RGBWColor(0, 255, 100, 0)), 0.1), // Tealish
    SHOOTHUB(new StrobeAnimation(0, 65).withColor(new RGBWColor(0, 255, 0, 0)), 0.1), // Green
    ZEROED(new StrobeAnimation(0,65).withColor(new RGBWColor(0, 0, 255, 100)), 0.1),//light blue   /**
    DISABLED(new StrobeAnimation(0,65).withColor(new RGBWColor(250, 0, 250, 100)), 0.1),//pink   /**
    DISCONNECTEDCAMERA(new StrobeAnimation(0,65).withColor(new RGBWColor(255,0,0,0)), 0.1),//violently red

    //NOT USING
    INTAKING(new StrobeAnimation(0, 65).withColor(new RGBWColor(0, 55, 255, 0)), 0), // Blue
    REVERSEINTAKING(new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 0, 255, 0)), 0); // The hottest pink



    /*  The strobe animation
>>>>>>> Stashed changes
   */
    StrobeAnimation animation;
    
    static RainbowAnimation rainbowAnim = new RainbowAnimation(0, 65);
       /**
       * The time
       */
        public double time;
    
    
        private LightAnimations(StrobeAnimation animation, double time) {
            this.animation = animation;
            this.time = time;
        }
        public static void RainbowAnim(){
           new RainbowAnimation(0, 65);
    }
    public static void Lights(){
        if(!Robot.zeroingLights){
            if(SwerveSubsystem.getInstance().isInAllianceZone()){
                if(Shooter.getInstance().isAimedAtHub && Shooter.getInstance().isHubActive()){
                    Lights.getLightInstance().lightsWantedState = SHOOTHUB;
                }else{
                    Lights.getLightInstance().lightsWantedState = CANTSHOOT;
                }
            }else{
                if(Shooter.getInstance().isAimedAtFerry){
                    Lights.getLightInstance().lightsWantedState = SHOOTFERRY;
                }else{
                    Lights.getLightInstance().lightsWantedState = CANTSHOOT;

                }
            }
        }else{
            Lights.getLightInstance().lightsWantedState = ZEROED;
        }

    }
}
