package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.Vision;

public class DriverCommunications {
    static boolean CurrentHubState = Shooter.getInstance().isHubActive();
    static String NextPhaseIndication = "Transition Period";
    static String PhaseIndication = "Autonomous";
    public static Field2d fieldPose = new Field2d();
    public static double PhaseClock = 0;
    public static double TeleopTimer = Timer.getMatchTime();
    static double[] lights = {0,0,0,0};
    public static double ShieldAdjustmentY = 0;
    public static double ShieldAdjustmentX = 0;
    public static void driverCommInit(){
        SmartDashboard.putNumber("Y adjustment", 0);
        SmartDashboard.putNumber("Y adjustment", 0);

    }

    static void allianceShift(int ShiftEndTime){
        PhaseClock = Math.floor(TeleopTimer - ShiftEndTime);
        if(!Shooter.getInstance().isHubActive()){
            NextPhaseIndication = "Our Shift";
            PhaseIndication = "Opposing Shift";
        }else{
            NextPhaseIndication = "Opposing Shift";
            PhaseIndication = "Our Shift";
        }

    }
    
    public static void pushToElastic() {
        if(Robot.TeleopStarted){ //if in teleop
            if (TeleopTimer > 130) { //in transition period
                PhaseIndication = "Transition Period";
                PhaseClock = Math.floor(TeleopTimer - 130);
                //Changing the "Next phase" indicator based on who won auto
                 if (Robot.WonAuto()){
                    NextPhaseIndication = "Opposing Shift";
                 }else{
                    NextPhaseIndication = "Our Shift";
                }

            }else if (TeleopTimer > 105) { //in alliance shift 1
                allianceShift(105);
            }else if (TeleopTimer > 80) { //in alliance shift 2
                allianceShift(80);
            }else if (TeleopTimer > 55) { //in alliance shift 3
                allianceShift(55);
            }else if (TeleopTimer > 30) { //in alliance shift 4
                PhaseClock = Math.floor(TeleopTimer - 30);
                NextPhaseIndication = "Endgame";
            }else{
                PhaseClock = Math.floor(TeleopTimer);
                NextPhaseIndication = "Match End";
                PhaseIndication = "Endgame";
            }

        }else{ //if in auto
            PhaseClock = Math.floor(TeleopTimer - 140);
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
        SmartDashboard.putNumber("teleopTimer", TeleopTimer);
        double[] defaultLights = {0, 0, 0, 0};
        Lights.manualLights = SmartDashboard.getNumberArray("RobotReveal/light Controller", defaultLights);
        ShieldAdjustmentY = SmartDashboard.getNumber("Y adjustment", 0);
        ShieldAdjustmentX = SmartDashboard.getNumber("X adjustment", 0);

        Lights.manualR = SmartDashboard.getNumber("RobotReveal/Manual Red", 255);
        Lights.manualG = SmartDashboard.getNumber("RobotReveal/Manual Green", 0);
        Lights.manualB = SmartDashboard.getNumber("RobotReveal/Manual Blue", 200);
        Lights.manualW = SmartDashboard.getNumber("RobotReveal/Manual White", 0);


        //SmartDashboard.putBoolean("In Manual?", Superstructure.getInstance().getCurrentState() == States.MANUAL);
        //SmartDashboard.putBoolean("Can Shoot", Shooter.getInstance().Shootable());
    }
}
