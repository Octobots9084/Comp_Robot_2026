package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Subsystem; // do we need this?
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import things to check in handleStateTransitions()

public class Climb extends SubsystemBase {
    ClimbStates currentState = ClimbStates.IDLE;
    public ClimbStates wantedState = ClimbStates.IDLE;
    public ClimbIO io;
    public static Climb instance;
    public boolean alreadyZeroed = false;
    public ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO io){
        this.io = io;
        instance = this;
    }

    public static Climb getInstance() {
        return instance;
    }

    public double getClimbPosition() {
        return io.getClimbPosition();
    }

    public double getDeployPosition() {
        return io.getDeployPosition();
    }

    public boolean climbInTolerance(double climbTolerance) {
        return io.climbInTolerance(climbTolerance);
    }

    public void setClimbState(ClimbStates state) {
        io.setClimbState(state);
    }

    public ClimbStates getClimbState() {
        return currentState;
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        applyStates();

        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case IDLE:
                if (currentState == ClimbStates.DEPLOYEDL1 || currentState == ClimbStates.DEPLOYEDL3) {
                    currentState = ClimbStates.IDLE;
                }
                break;
            case DEPLOYEDL1:
                if (currentState == ClimbStates.IDLE || currentState == ClimbStates.CLIMBEDL1) {
                    currentState = ClimbStates.DEPLOYEDL1;
                }
                break;
            case CLIMBEDL1:
                if (currentState == ClimbStates.DEPLOYEDL1) {
                    currentState = ClimbStates.CLIMBEDL1;
                }
                break;
            case DEPLOYEDL3:
                if (currentState == ClimbStates.IDLE || currentState == ClimbStates.ENGAGEDL3) {
                    currentState = ClimbStates.DEPLOYEDL3;
                }
                break;
            case ENGAGEDL3:
                if (currentState == ClimbStates.DEPLOYEDL3 || currentState == ClimbStates.CLIMBEDL3) {
                    currentState = ClimbStates.ENGAGEDL3;
                }
                break;
            case CLIMBEDL3:
                if (currentState == ClimbStates.ENGAGEDL3) {
                    currentState = ClimbStates.CLIMBEDL3;
                }
                break;
            case ZERO:
                if (currentState != ClimbStates.CLIMBEDL3 && currentState != ClimbStates.CLIMBEDL1) {
                    currentState = ClimbStates.ZERO;
                }
                break;

        }

    }

    public void applyStates() {
        switch (currentState) {
            case ZERO:
                if (zeroClimb()) {
                    alreadyZeroed = true;
                    wantedState = ClimbStates.IDLE;
                }
                break;
            default:
                setClimbState(currentState);
                break;

        }
    }

    public boolean zeroClimb() {
        boolean ifPressed = io.isZeroingSwitchPressed();
        if (ifPressed) {
            io.setRotateVoltage(0);
        } else {
            io.setRotateVoltage(-3);
        }
        return ifPressed;
    }
}
