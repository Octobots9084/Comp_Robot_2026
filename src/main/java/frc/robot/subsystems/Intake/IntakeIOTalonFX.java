package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
     public DigitalInput zeroingSwitch = new DigitalInput(0);
     public IntakeConfigurator config;
     public TalonFX pivot;
     public TalonFX roller;
     private MotionMagicVelocityVoltage rollerRequest = new MotionMagicVelocityVoltage(0);
     private MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

     public IntakeIOTalonFX() {
          config = new IntakeConfigurator();

          roller = new TalonFX(Constants.intakeRollerID, Constants.krakenBus);
          pivot = new TalonFX(Constants.intakePivotID, Constants.krakenBus);
          roller.setNeutralMode(NeutralModeValue.Coast);
          roller.getConfigurator().apply(config.intakeRollerConfig);
          pivot.getConfigurator().apply(config.intakePivotConfig);
     }

     public void updateInputs(IntakeIOInputs inputs) {
          inputs.intakePosition = pivot.getPosition().getValueAsDouble();
          inputs.rollerRPS = roller.getVelocity().getValueAsDouble();
          inputs.rollerTemp = roller.getDeviceTemp().getValueAsDouble();
          inputs.pivotTemp = pivot.getDeviceTemp().getValueAsDouble();
     }

     @Override
     public void setIntakeState(IntakeStates states) {
          // pivotRequest.Position = states.intakePosition;
          pivot.setControl(pivotRequest.withPosition(states.intakePosition));
          roller.setVoltage(states.rollerVoltage);
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

     public boolean isZeroingSwitchPressed() {// GAS_D
          return zeroingSwitch.get();
     }

         public boolean zeroIntake() {
        boolean pressed = isZeroingSwitchPressed();
        if (!pressed) {
            setRotateVoltage(0);
            pivot.setPosition(0);
        } else {

        }
        return !pressed;
    }
}
