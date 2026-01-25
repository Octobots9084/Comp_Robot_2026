package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO{
    //public TalonFX intakePivotMotor;
    //public TalonFX intakeRollerMotor;
    //do intake configs
    public IntakeConfigurator intakeConfigs;
    private MotionMagicVoltage intakePivotRequest;
    private MotionMagicVelocityVoltage intakeRollerRequest;

    public IntakeIOTalonFX(){
        intakeConfigs = new IntakeConfigurator();
        //intakePivotMotor = new TalonFX(IntakeConstants.intakePivotID, "Default Name");
        //intakeRollerMotor = new TalonFX(IntakeConstants.intakeRollerID, "Default Name");

    //    intakePivotMotor.getConfigurator().apply(intakeConfigs.intakePivotConfig);
      //  intakeRollerMotor.getConfigurator().apply(intakeConfigs.intakeRollerConfig);
    }

    @Override
    public void setIntakeState(IntakeStates state){
        intakePivotRequest.Position = state.intakePosition;
        intakeRollerRequest.Velocity = state.rollerRPS;
        //intakePivotMotor.setControl(intakePivotRequest);
        //intakeRollerMotor.setControl(intakeRollerRequest);
    }

    @Override
    public double getPivotPosition(){
        return 0;//intakePivotMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getIntakeVelocity(){
        return 0;//intakeRollerMotor.getVelocity().getValueAsDouble();
    }

    @Override 
    public boolean pivotInTolerance(double tolerance){
        return true;//MathUtil.isNear(intakePivotRequest.getPositionMeasure().in(Units.Revolution), this.getPivotPosition(), tolerance);
    }

    @Override 
    public boolean rollerInTolerance(double tolerance){
        return MathUtil.isNear(intakeRollerRequest.getVelocityMeasure().in(Units.RadiansPerSecond), this.getIntakeVelocity(), tolerance);
    }

     @Override 
    public boolean intakeInTolerance(double tolerance){
        return rollerInTolerance(tolerance) && pivotInTolerance(tolerance);
    }
}
