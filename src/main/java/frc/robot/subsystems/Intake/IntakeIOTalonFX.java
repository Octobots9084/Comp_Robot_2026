package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO{
//     public DigitalInput zeroingSwitch = new DigitalInput(9);
    public IntakeConfigurator config;
//     public TalonFX pivot;
//     public TalonFX roller;
    private MotionMagicVelocityVoltage rollerRequest = new MotionMagicVelocityVoltage(0);
    private MotionMagicVoltage pivotRequest;

    public IntakeIOTalonFX () {
        config = new IntakeConfigurator();
     //    roller.setNeutralMode(NeutralModeValue.Coast);

     //    roller = new TalonFX(Constants.intakeRollerID, Constants.krakenBus);
     //    pivot = new TalonFX(Constants.intakePivotID, Constants.krakenBus);
     //    roller.getConfigurator().apply(config.intakeRollerConfig);
     //    pivot.getConfigurator().apply(config.intakePivotConfig);
    }

   public void updateInputs(IntakeIOInputs inputs) {
     //    inputs.intakePosition = pivot.getPosition().getValueAsDouble();
     //    inputs.rollerRPS = roller.getVelocity().getValueAsDouble();
     // //    inputs.rollerTemp = roller.getDeviceTemp().getValueAsDouble();
     //    inputs.pivotTemp = pivot.getDeviceTemp().getValueAsDouble();
   }

    @Override
   public void setIntakeState(IntakeStates states){
     //    pivotRequest.Position = states.intakePosition;
     //    rollerRequest.Velocity = states.rollerRPS;
     //    pivot.setControl(pivotRequest);
     //    roller.setControl(rollerRequest);
          // roller.setVoltage(states.rollerRPS/6);
   }

   @Override
   public double getRollerRPS() {
        return -1;// roller.getVelocity().getValueAsDouble();
   }

   @Override
   public double getIntakePosition() {
        return -1; // pivot.getPosition().getValueAsDouble(); 
   }
   public void setRotateVoltage(double voltage){
     //    this.pivot.setVoltage(voltage);
    }
//    public boolean isZeroingSwitchPressed(){//GAS_D
//          return zeroingSwitch.get();
//    }
}
