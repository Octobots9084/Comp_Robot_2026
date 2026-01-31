package frc.robot.subsystems.Lights;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.StrobeAnimation;
public enum LightAnimations{
    DEFAULT(new StrobeAnimation(0,0).withColor(new RGBWColor(255,255,0,0)), 0),             // Yellow
    CANTSHOOT(new StrobeAnimation(0,0).withColor(new RGBWColor(255,120,0,0)), 0),                 // Orange
    INTAKING(new StrobeAnimation(0,0).withColor(new RGBWColor(0,55,255,0)), 0),             // Blue
    REVERSEINTAKING(new StrobeAnimation(0,0).withColor(new RGBWColor(255,0,255,0)), 0),     // The hottest pink
    SHOOTREADYMANUAL(new StrobeAnimation(0,0).withColor(new RGBWColor(0,255,100,0)), 0),    // Green
    SHOOTREADYCONTINIOUS(new StrobeAnimation(0,0).withColor(new RGBWColor(0,200,255,0)),0); // Teal
     private StrobeAnimation animation;
     public double time;
         private LightAnimations(StrobeAnimation animation, double time) {
                      this.animation = animation;
        this.time = time;
    }
}
    
