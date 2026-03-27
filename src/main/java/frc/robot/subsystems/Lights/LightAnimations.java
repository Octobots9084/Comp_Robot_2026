package frc.robot.subsystems.Lights;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

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
    DEFAULT(new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 255, 0, 0)), 1), // Yellow
    CANTSHOOT(new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 0, 0, 0)), 1), // Red
    INTAKING(new StrobeAnimation(0, 65).withColor(new RGBWColor(0, 55, 255, 0)), 1), // Blue
    REVERSEINTAKING(new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 0, 255, 0)), 1), // The hottest pink
    SHOOTFERRY(new StrobeAnimation(0, 65).withColor(new RGBWColor(0, 255, 100, 0)), 1), // Green
    SHOOTHUB(new StrobeAnimation(0, 65).withColor(new RGBWColor(0, 200, 255, 0)), 1), // Teal
    ZEROED(new StrobeAnimation(0,65).withColor(new RGBWColor(250, 0, 0, 100)), 1);//light red   /**
/*  The strobe animation
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

}
