package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter.Shooter;

public class DriverCommunications{
    public static void pushToElastic(){
              SmartDashboard.putNumber("Phase Shift Countdown", 0 );
              SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
              SmartDashboard.putBoolean("Is Hub Active?", Shooter.getInstance().isHubActive());
              SmartDashboard.putBoolean("In Manual?", Superstructure.getInstance().getCurrentState() == States.MANUAL);
              SmartDashboard.putBoolean("Can Shoot", Shooter.getInstance().Shootable());
               double PhaseTime = 0;
               Timer PhaseCountdown = new Timer();
                if(Constants.timer.get() == 0){
                    PhaseTime = 10;
                }
    }

    }

