package frc.robot.subsystems.Lights;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;




   /**
   * The premade color and strobe settings for {@link frc.robot.subsystems.Lights.Lights Lights}
   * <br></br>
   * <b>Colors</b>
   * <ul>
   * <li>DEFAULT - Yellow</li>
   * <li>CANTSHOOT - Orange</li>
   * <li>INTAKING - Blue</li>
   * <li>REVERSEINTAKING - Pink</li>
   * <li>SHOOTREADYMANUAL - Green</li>
   * <li>SHOOTREADYCONTINIOUS - Teal</li>
   * </ul>
   */
public enum LightAnimations {

    DEFAULT(new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 255, 0, 0)), 0.1), // Yellow
    CANTSHOOT(new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 20, 0, 0)), 0.1), // Orange
    SHOOTFERRY(new StrobeAnimation(0, 65).withColor(new RGBWColor(0, 255, 100, 0)), 0.1), // Green
    SHOOTHUB(new StrobeAnimation(0, 65).withColor(new RGBWColor(0, 200, 255, 0)), 0.1), // Teal
    ZEROED(new StrobeAnimation(0,65).withColor(new RGBWColor(250, 0, 0, 100)), 0.1),//light blue   /**
    DISABLED(new StrobeAnimation(0,65).withColor(new RGBWColor(250, 100, 0, 100)), 0.1),//light blue   /**
    DISCONNECTEDCAMERA(new StrobeAnimation(0,65).withColor(new RGBWColor(255,0,0,0)), 0.1),

    //NOT USING
    INTAKING(new StrobeAnimation(0, 65).withColor(new RGBWColor(0, 55, 255, 0)), 0), // Blue
    REVERSEINTAKING(new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 0, 255, 0)), 0); // The hottest pink



    /*  The strobe animation
>>>>>>> Stashed changes
   */
    StrobeAnimation animation;
    
    static RainbowAnimation rainbowAnim = new RainbowAnimation(0, 64);
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
        if(SwerveSubsystem.getInstance().isInAllianceZone()){
            if(Shooter.getInstance().isAimedAtHub){
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
    }
}
