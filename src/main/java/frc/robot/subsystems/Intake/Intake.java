package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lights.LightAnimations;
import frc.robot.subsystems.Lights.Lights;

// import frc.robot.subsystems.Lights;
public class Intake extends SubsystemBase {
    /**
   * Tells the robot to intake during autonomous.
   *
   * @param True Sets the wantedState to INTAKING
   * @param False Does nothing
   */
    // public boolean autonomousIntake = false;
    // public IntakeStates autoToTeleopState = IntakeStates.SAFE;
 /**
   * The current state of the intake, which determines what the intake does
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Intake.IntakeStates#SAFE SAFE}
   * @param IntakeStates The intake states contain roller speed and intake position - {@link frc.robot.subsystems.Intake.IntakeStates IntakeStates}
   */
  
    public IntakeStates currentState = IntakeStates.SAFE;
     /**
   * The wanted state of the intake, which the subsystem attempts to set the {@link #currentState} to
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Intake.IntakeStates#ZERO ZERO}
   * @param IntakeStates The intake states contain roller speed and intake position - {@link frc.robot.subsystems.Intake.IntakeStates IntakeStates}
   */
    public IntakeStates wantedState = IntakeStates.ZERO;
    public IntakeIO io;
    public static Intake instance;
    public boolean alreadyZeroed = false;
    public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    public int elephantiaissTimer = 0;
    public boolean autoIntaked = false;

    public Intake(IntakeIO io) {
        this.io = io;
        instance = this;

    }

    public static Intake getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        // if (autonomousIntake) {
        //     wantedState = IntakeStates.INTAKING;
        // }only if it intake auto doesnt work
        // This will handle changing between states at the user's request.
        handleStateTransitions();

        // this is where states actually take effect.
        applyStates();
        
        inputs.currentState = currentState;
        inputs.wantedState = wantedState;
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void handleStateTransitions() {

        switch (wantedState) {
            case SAFE:
                currentState = IntakeStates.SAFE;
                break;

            case INTAKING:
            if (currentState != IntakeStates.ZERO || alreadyZeroed == true) {
                // only works if not climbing
                currentState = IntakeStates.INTAKING;
            }
                // Lights.getLightInstance().lightsWantedState = LightAnimations.INTAKING;
                break;

            case EXTENDED:
                // only works if not climbing
                currentState = IntakeStates.EXTENDED;
                break;

            case REVERSEINTAKING:
                // only works if not climbing
                currentState = IntakeStates.REVERSEINTAKING;
                // Lights.getLightInstance().lightsWantedState =
                // LightAnimations.REVERSEINTAKING;

                break;
            case ZERO:
                    currentState = IntakeStates.ZERO;
                break;
            case ELEPHANTIASISPART2:
                    if (currentState!=IntakeStates.ELEPHANTIASISPART2){
                        elephantiaissTimer = 0;
                        currentState = IntakeStates.ELEPHANTIASISPART2;
                    }
                break;
            default:
                currentState = IntakeStates.SAFE;
                break;

        }
    }

    public void applyStates() {

        switch (currentState) {

            case INTAKING:
            //motors on intake out
            io.setIntakeState(currentState);
            //Lights.getLightInstance().lightsWantedState = LightAnimations.INTAKING;
            break;

            case EXTENDED:
            //motors off intake out
            io.setIntakeState(currentState);
                
            break;

            case SAFE:
            //motors off intake in
            io.setIntakeState(currentState);
            break;

            case REVERSEINTAKING:
            io.setIntakeState(currentState);
            //Lights.getLightInstance().lightsWantedState = LightAnimations.REVERSEINTAKING;
            break;
            case ZERO:
                if (io.zeroIntake()) {
                    alreadyZeroed = true;
                    if (!DriverStation.isAutonomousEnabled()) {
                        wantedState = IntakeStates.EXTENDED;
                    }
                }else{
                    alreadyZeroed = false;
                }
                break;
            case ELEPHANTIASISPART2:
                if (elephantiaissTimer<0){
                    io.setIntakeState(IntakeStates.INTAKING);
                } else {
                    io.setIntakeState(IntakeStates.PARTIALEXTENTION);
                }


                elephantiaissTimer++;
                if (elephantiaissTimer > 20){
                    elephantiaissTimer = -20;
                }
                break;
        default:
            io.setIntakeState(currentState);    
            break;
        }
    }

    public void setCurrentState(IntakeStates state) {
        this.currentState = state;
    }

    public IntakeStates getCurrentState() {
        return this.currentState;
    }

    public void setWantedState(IntakeStates state) {
        this.wantedState = state;
    }

    public IntakeStates getWantedState() {
        return this.wantedState;
    }
}
