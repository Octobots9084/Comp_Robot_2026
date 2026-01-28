package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX; // I think this is important...
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Unit;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Revolutions;

import com.ctre.phoenix6.controls.Follower;

public class ClimbIOTalonFX implements ClimbIO{
    //controlls the climb motor rotate (follower is influenced by extreiror varible)
    public TalonFX climbRotateMotorControlled;
    public TalonFX climbRotateMotorFollower;
    //the deployment varible for deployment motor
    public TalonFX climbDeploymentMotor;

    public ClimbConfigurator climbConfig; 

    //create two MotionMagicVoltage variables for each of the controlled motors, the followed 
    private MotionMagicVoltage climbMotionControlledRequest;
    private MotionMagicVoltage climbDeployRequest;

    public ClimbIOTalonFX() {
        climbConfig = new ClimbConfigurator();
        
        climbRotateMotorControlled = new TalonFX(Constants.climbRotateControlledID, Constants.krakenBus);
        climbRotateMotorFollower = new TalonFX(Constants.climbRotateFollowerID, Constants.krakenBus);
        climbDeploymentMotor = new TalonFX(Constants.climbDeployID, Constants.krakenBus);

        climbRotateMotorControlled.getConfigurator().apply(climbConfig.climbRotateControlledConfig);
        climbRotateMotorFollower.getConfigurator().apply(climbConfig.climbDeployConfig);
        climbDeploymentMotor.getConfigurator().apply(climbConfig.climbDeployConfig);
    
        climbMotionControlledRequest = new MotionMagicVoltage(0.0);
        climbDeployRequest = new MotionMagicVoltage(0.0);
    
    }
    @Override
    public void updateInputs(ClimbIOInputs inputs){
        inputs.climbMotorControlledTemperature = climbRotateMotorControlled.getDeviceTemp().getValueAsDouble();
        inputs.climbMotorFollowerTemperature = climbRotateMotorFollower.getDeviceTemp().getValueAsDouble();
        inputs.deployMotorTemperature = climbDeploymentMotor.getDeviceTemp().getValueAsDouble();
        inputs.climbPosition = climbRotateMotorControlled.getPosition().getValueAsDouble();
        inputs.deployPosition = climbRotateMotorControlled.getPosition().getValueAsDouble();
        
        
    }

    @Override
    public void setClimbState(ClimbStates state) {
        climbMotionControlledRequest.Position = state.climbPosition;
        climbRotateMotorControlled.setControl(climbMotionControlledRequest);
        climbRotateMotorFollower.setControl(new Follower(Constants.climbRotateControlledID, MotorAlignmentValue.Opposed));
    
        
        climbDeployRequest.Position = state.climbPosition;
        climbDeploymentMotor.setControl(climbDeployRequest);
    
    }
    @Override
    public double getClimbPosition(){
        return climbRotateMotorControlled.getPosition().getValueAsDouble();
    }

    @Override
    public double getDeployPosition(){
        return climbDeploymentMotor.getPosition().getValueAsDouble();
    }
    @Override
    public boolean climbInTolerance(double climbTolerance){
        return MathUtil.isNear(climbMotionControlledRequest.getPositionMeasure().in(Revolutions), this.getClimbPosition(), climbTolerance);
    }
}