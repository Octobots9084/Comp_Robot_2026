package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
     public DigitalInput zeroingSwitch = new DigitalInput(0);
     public IntakeConfigurator config;
     public TalonFX pivot;
     public TalonFX roller;
     public TalonFX pivotfollower;
     public TalonFX rollerfollower;
     private VelocityVoltage rollerRequest = new VelocityVoltage(0);
     private MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

     private Follower followPivot = new Follower(Constants.intakePivotID, MotorAlignmentValue.Opposed);
     private Follower followRoller = new Follower(Constants.intakeRollerID, MotorAlignmentValue.Opposed);

     public IntakeIOTalonFX() {
          config = new IntakeConfigurator();
          
          roller = new TalonFX(Constants.intakeRollerID, Constants.krakenBus);
          pivot = new TalonFX(Constants.intakePivotID, Constants.krakenBus);

          rollerfollower = new TalonFX(Constants.intakeRollerFollowerID, Constants.krakenBus);
          pivotfollower = new TalonFX(Constants.intakePivotFollowerID, Constants.krakenBus);

          roller.setNeutralMode(NeutralModeValue.Coast);

          roller.getConfigurator().apply(config.intakeRollerConfig);
          pivot.getConfigurator().apply(config.intakePivotConfig);
     }

     public void updateInputs(IntakeIOInputs inputs) {
          inputs.intakePosition = pivot.getPosition().getValueAsDouble();
          inputs.rollerRPS = roller.getVelocity().getValueAsDouble();
          inputs.pivotRequest = pivotRequest.getPositionMeasure().in(Units.Rotations);
          // inputs.pivotCurrent = pivot.getStatorCurrent().getValueAsDouble();

          inputs.pivotLimitSwitch = this.isZeroingSwitchPressed();
     }

     @Override
     public void setIntakeState(IntakeStates states) {
          // pivotRequest.Position = states.intakePosition;
          pivot.setControl(pivotRequest.withPosition(states.intakePosition));
          roller.setControl(rollerRequest.withVelocity(states.rollerRPS));

          pivotfollower.setControl(followPivot);
          rollerfollower.setControl(followRoller);

     }

     @Override
     public double getRollerRPS() {
          return roller.getVelocity().getValueAsDouble();
     }

     @Override
     public double getIntakePosition() {
          return pivot.getPosition().getValueAsDouble();
     }
     @Override
     public void setRotateVoltage(double voltage) {
          this.pivot.setVoltage(voltage);
     }

     @Override
     public void setRollerSpeed(double rps){
          roller.setControl(rollerRequest.withVelocity(rps));
     }

     public boolean isZeroingSwitchPressed() {
          return zeroingSwitch.get();
     }

     public boolean zeroIntake() {
        boolean pressed = isZeroingSwitchPressed();
        if (!pressed) {
            setRotateVoltage(0);
            pivot.setPosition(0);
            Intake.getInstance().alreadyZeroed = true;
        } else {
          setRotateVoltage(-2);
          Intake.getInstance().alreadyZeroed = false;
        }
        return !pressed;
    }
}
