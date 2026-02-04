package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO{
    public IntakeConfigurator config;
    public TalonFX roller;
    public TalonFX pivot;
    private MotionMagicVelocityVoltage rollerRequest = new MotionMagicVelocityVoltage(0);
    private MotionMagicVoltage pivotRequest;

    public IntakeIOTalonFX () {
        config = new IntakeConfigurator();

        roller = new TalonFX(Constants.intakeRollerID, Constants.krakenBus);
        pivot = new TalonFX(Constants.intakePivotID, Constants.krakenBus);
    }

   public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition = pivot.getPosition().getValueAsDouble();
        inputs.rollerRPS = roller.getVelocity().getValueAsDouble();
        inputs.rollerTemp = roller.getDeviceTemp().getValueAsDouble();
        inputs.pivotTemp = pivot.getDeviceTemp().getValueAsDouble();
   }

    @Override
   public void setIntakeState(IntakeStates states){
     //    pivotRequest.Position = states.intakePosition;
        rollerRequest.Velocity = states.rollerRPS;
     //    pivot.setControl(pivotRequest);
        roller.setControl(rollerRequest);
   }

   @Override
   public double getRollerRPS() {
        return roller.getVelocity().getValueAsDouble();
   }

   @Override
   public double getIntakePosition() {
        return pivot.getPosition().getValueAsDouble(); 
   }


}
