package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;

public class DriverCommunications {
    static boolean CurrentHubState = Shooter.getInstance().isHubActive();
    static String NextPhaseIndication = "Transition Period";
    static double PhaseTime = 0;
    public static Field2d fieldPose = new Field2d();
        static Timer PhaseCountdown = new Timer();
        public static void pushToElastic() {
    
    
            // If the hub state changes, reset the phase shift timer and change
            // currentHUbSTate
            if (!Shooter.getInstance().isHubActive() == CurrentHubState) {
                CurrentHubState = Shooter.getInstance().isHubActive();
                PhaseCountdown.restart();
                if ((Constants.timer.get() >= 28) && (Constants.timer.get() < 103)) {
                    if (NextPhaseIndication == "Opposing Shift"){
                        NextPhaseIndication = "Our Shift";
                    }else{
                        NextPhaseIndication = "Opposing Shift";
    
                    }
                }else if ((Constants.timer.get() >= 103) && (Constants.timer.get() < 128)) {
                   NextPhaseIndication = "Endgame";
                }
            }

            if ((Constants.timer.get() >= 130) && (Constants.timer.get() <= 130.5)) {
                PhaseTime = 30;
                PhaseCountdown.restart();
            }
            if (((Constants.timer.get() >= 20) && (Constants.timer.get() <= 20.5))) {
                PhaseCountdown.restart();
                PhaseTime = 10;

            }
            if ((Constants.timer.get() <= 20)) {
                PhaseTime = 20;
                NextPhaseIndication = "Transition Period";
        } else if ((Constants.timer.get() > 20) && (Constants.timer.get() <= 30)) {
                PhaseTime = 10;
                //Changing the "Next phase" indicator based on who won auto
                 if (Robot.WonAuto()){
                    NextPhaseIndication = "Opposing Shift";
                 }else{
                    NextPhaseIndication = "Our Shift";
             }
        } else if ((Constants.timer.get() > 30) && (Constants.timer.get() < 130)) {
            PhaseTime = 25;

        }else if ((Constants.timer.get() >= 130)){
            PhaseTime = 30;
            NextPhaseIndication = "Match End";
        }

        // set PhaseClock as the time before phase shift by subtracting timer from max
        // shift time
        double PhaseClock = (PhaseTime - PhaseCountdown.get());
        SmartDashboard.putString("Next Phase", NextPhaseIndication);
        SmartDashboard.putNumber("Phase Shift Countdown", PhaseClock);
        SmartDashboard.putData("Field", fieldPose);
        SmartDashboard.putNumber("Timer", Constants.timer.get());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Is Hub Active?", Shooter.getInstance().isHubActive());
        //SmartDashboard.putBoolean("In Manual?", Superstructure.getInstance().getCurrentState() == States.MANUAL);
        // SmartDashboard.putBoolean("Can Shoot", Shooter.getInstance().Shootable());
    }
}
