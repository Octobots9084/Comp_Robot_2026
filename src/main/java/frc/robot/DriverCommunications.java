package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.Vision;

public class DriverCommunications {
    static boolean CurrentHubState = Shooter.getInstance().isHubActive();
    static String NextPhaseIndication = "Transition Period";
    static String PhaseIndication = "Autonomous";
    static double TeleopAccounted = 1.6;
    static double PhaseClock = 0;
    public static Field2d fieldPose = new Field2d();
    public static double TeleopTimer = Constants.timer.get() - TeleopAccounted;
    static void allianceShift(int ShiftEndTime){
        PhaseClock = Math.round(ShiftEndTime - TeleopTimer);
        if(!Shooter.getInstance().isHubActive()){
            NextPhaseIndication = "Our Shift";
            PhaseIndication = "Opposing Shift";
        }else{
            NextPhaseIndication = "Opposing Shift";
            PhaseIndication = "Our Shift";
        }

    }
    
    public static void pushToElastic() {
        TeleopTimer = Constants.timer.get() - TeleopAccounted;
        if(Robot.TeleopStarted){ //if in teleop
            if (TeleopTimer < 10) { //in transition period
                PhaseClock = Math.round(10 -TeleopTimer);
                PhaseIndication = "Transition Period";
                //Changing the "Next phase" indicator based on who won auto
                 if (Robot.WonAuto()){
                    NextPhaseIndication = "Opposing Shift";
                 }else{
                    NextPhaseIndication = "Our Shift";
                }

            }else if (TeleopTimer < 35) { //in alliance shift 1
                allianceShift(35);
            }else if (TeleopTimer < 60) { //in alliance shift 2
                allianceShift(60);
            }else if (TeleopTimer < 85) { //in alliance shift 3
                allianceShift(85);
            }else if (TeleopTimer < 110) { //in alliance shift 4
                NextPhaseIndication = "Endgame";
                PhaseClock = Math.round(110 - TeleopTimer);
            }else{
                PhaseClock = Math.round(140 - TeleopTimer);
                NextPhaseIndication = "Match End";
                PhaseIndication = "Endgame";
            }

        }else{ //if in auto
            PhaseClock = Math.round(20 - TeleopTimer);
            NextPhaseIndication = "Transition Phase";
            PhaseIndication = "Autonomous";

        }

        // set PhaseClock as the time before phase shift by subtracting timer from max
        // shift time
        SmartDashboard.putString("Next Phase", NextPhaseIndication);
        SmartDashboard.putString("Current Phase", PhaseIndication);
        SmartDashboard.putNumber("Phase Shift Countdown", PhaseClock);
        SmartDashboard.putData("Field", fieldPose);
        SmartDashboard.putNumber("Timer", Constants.timer.get());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Hub Activity", Shooter.getInstance().isHubActive());
        SmartDashboard.putBoolean("Cam/Front Right", Vision.getInstance().io.CameraConnect(0));
        SmartDashboard.putBoolean("Cam/Front Left", Vision.getInstance().io.CameraConnect(1));
        SmartDashboard.putBoolean("Cam/Left", Vision.getInstance().io.CameraConnect(2));
        SmartDashboard.putBoolean("Cam/Right", Vision.getInstance().io.CameraConnect(3));
        SmartDashboard.putBoolean("Cam/Back", Vision.getInstance().io.CameraConnect(4));





        //SmartDashboard.putBoolean("In Manual?", Superstructure.getInstance().getCurrentState() == States.MANUAL);
        //SmartDashboard.putBoolean("Can Shoot", Shooter.getInstance().Shootable());
    }
}
