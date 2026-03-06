package frc.robot.subsystems;

import java.security.spec.ECPublicKeySpec;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.auto.StateChange.SetIntakeStateSafe;
import frc.robot.subsystems.*; // WHY DID WE HAVE SO MANY IMPORTS FROM THIS THING JUST IMPORT IT ALL
import frc.robot.subsystems.Climb.*; //I don't know why we need this
import frc.robot.subsystems.Drive.SwerveStates;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.*;//same
import frc.robot.subsystems.Shooter.*;//same here
import frc.robot.subsystems.Shooter.Feeder.Feeder;
import frc.robot.subsystems.Shooter.Flywheel.*;
import frc.robot.subsystems.Shooter.Turret.Turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

public class Superstructure extends SubsystemBase {
    public States currentState = States.SAFE;
    public States wantedState = States.SAFE;
    public States prevState = States.SAFE;
    public IntakeStates userRequestedIntakeState = IntakeStates.SAFE;

    boolean climbDescending = true;
    boolean climbAligned = false;
    public static Superstructure currentInstance;
    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    // public Climb climb = Climb.getInstance();
    public Shooter shooter = Shooter.getInstance();
    public Intake intake = Intake.getInstance();

    @Override
    public void periodic() {
        prevState = currentState;
        Logger.recordOutput("prevState", this.prevState);
        Logger.recordOutput("currentState", this.currentState);
        Logger.recordOutput("wantedState", this.wantedState);
        Logger.recordOutput("climbDescending", this.climbDescending);
        handleStateTransitions();
        applyStates();
    }

    public Superstructure() {
        currentInstance = this;
    }

    public static void setInstance(Superstructure instance) {
        currentInstance = instance;
    }

    public static Superstructure getInstance() {
        // if(currentInstance == null){
        // throw new IllegalStateException("Superstructure Instance not set");
        // }
        return currentInstance;
    }

    public States getCurrentState() {
        return currentState;
    }

    public States getWantedState() {
        return wantedState;
    }

    public void setCurrentState(States state) {
        currentState = state;
    }

    public void setWantedState(States state) {
        wantedState = state;
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case SAFE:
                // if (climb.getClimbState() != ClimbStates.CLIMBEDL1 || climb.getClimbState() != ClimbStates.CLIMBEDL3) {
                    currentState = States.SAFE;
                // }
                break;
            case CLIMB_L3:
                currentState = States.CLIMB_L3;
                break;
            case CLIMB_L1:
                currentState = States.CLIMB_L1;
                break;
            case SHOOTER:
                // if (climb.getClimbState() != ClimbStates.CLIMBEDL1 || climb.getClimbState() != ClimbStates.CLIMBEDL3) {
                    currentState = States.SHOOTER;
                    
                // }
                break;
            case ZERO:
                if (!shooter.turretAlreadyZeroed || !intake.alreadyZeroed){
                    currentState = States.ZERO;
                }
                else if(DriverStation.isAutonomousEnabled()){
                    wantedState = States.AUTONONFIRE;
                }
                else {
                    wantedState = States.SHOOTER;
                }
                break;
            case AUTO:
                if(DriverStation.isAutonomous()){
                    this.currentState = States.AUTO;
                }
                break;
            case AUTONONFIRE:
                if(DriverStation.isAutonomous()){
                    this.currentState = States.AUTONONFIRE;
                }
            break;
            default:
                break; // do nothing
        }
    }

    public void applyStates() {
        switch (currentState) {
            case SAFE:
                stateSAFE();
                break;
            case MANUAL:
                stateMANUAL();
                break;
            case CLIMB_L3:
                stateCLIMBL3();
                break;
            case CLIMB_L1:
                stateCLIMBL1();
                break;
            case SHOOTER:
                stateSHOOTER();
                break;
            case ZERO:
                if(stateZERO()){
                    if(DriverStation.isAutonomousEnabled()){
                        wantedState = States.AUTONONFIRE;
                    }else{
                        wantedState = States.SHOOTER;
                        shooter.wantedShooterState = ShooterStates.HUB;
                    }
                }
                break;
            case AUTONONFIRE:
                swerve.wantedState = SwerveStates.IDLE;
                shooter.wantedShooterState = ShooterStates.SAFE;
                break;
            case AUTO:
                swerve.wantedState = SwerveStates.IDLE;
                shooter.wantedShooterState = ShooterStates.AUTOHUB;
                break;
            default:
                // throw an exception
                break;
        }
    }

    private void stateSAFE() {
        // climb.setClimbState(ClimbStates.IDLE);
        // // set shooter into safe state
        // intake.setWantedState(IntakeStates.SAFE);
        shooter.wantedShooterState = ShooterStates.SAFE;

    }
    private void stateMANUAL() {
        // TODO map buttons to direct inputs
        // turn off intake when starting
        // turn off shooter when starting
    }
    private void stateCLIMBL3() {
        stowForClimb();
        // climb.setClimbState(ClimbStates.DEPLOYEDL3);
        // // TODO align to bar(use button before alignment)
        // climb.setClimbState(ClimbStates.ENGAGEDL3);
        // //TODO align to vertical pole(button before alignment)
        // climb.setClimbState(ClimbStates.CLIMBEDL3);
        boolean climbAligned = true; //TODO when rui finishes alignment put this when it finishes

    }
    private void stateCLIMBL1() {
        stowForClimb();
        // climb.setClimbState(ClimbStates.DEPLOYEDL1);
        // //TODO align to bar(button before alignment)
        // climb.setClimbState(ClimbStates.CLIMBEDL1);
    }
   

    private void stateSHOOTER() {
        // if (userRequestedIntakeState != Intake.getInstance().currentState) {
        //     Intake.getInstance().wantedState = userRequestedIntakeState;
        // }
        swerve.wantedState = SwerveStates.MANUAL;
        // if(prevState != States.SHOOTER){
        //     shooter.wantedShooterState = ShooterStates.HUB;
        //     prevState = States.SHOOTER;

        // }
        shooter.wantedShooterState = ShooterStates.HUB;

        
    }

    private boolean stateZERO(){
        swerve.wantedState = SwerveStates.MANUAL;
        shooter.wantedShooterState = ShooterStates.ZERO;
        intake.wantedState = IntakeStates.ZERO; 
        // climb.wantedState = ClimbStates.ZERO;
        return (shooter.turretAlreadyZeroed && intake.alreadyZeroed);//&& climb.alreadyZeroed);
    }

    public void stowForClimb(){
        //   Intake.getInstance().setWantedState(IntakeStates.SAFE);
          Shooter.getInstance().wantedShooterState = ShooterStates.SAFE;
          
    }
}