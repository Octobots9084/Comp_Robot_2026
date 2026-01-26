package frc.robot.subsystems.Lights;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.StrobeAnimation;
public enum LightAnimations{
    DEFAULT(new StrobeAnimation(255, 255, 255, 0, 1, Constants.NUM_LEDS), 0),
    BUMP,
    INTAKING,
    SHOOTING,;
    
     private StrobeAnimation animation;
     public double time;
         private LightAnimations(StrobeAnimation animation, double time) {
                      this.animation = animation;
        this.time = time;
    }
            LightAnimations() {
                //TODO Auto-generated constructor stub
            }
        
}
    
