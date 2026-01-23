package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX; // I think this is important...
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Unit;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.GeneralConstants;

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
        
        climbRotateMotorControlled = new TalonFX(ClimbConstants.climbRotateControlledID, GeneralConstants.krakenBus);
        climbRotateMotorFollower = new TalonFX(ClimbConstants.climbRotateFollowerID, GeneralConstants.krakenBus);
        climbDeploymentMotor = new TalonFX(ClimbConstants.climbDeployID, GeneralConstants.krakenBus);

        climbRotateMotorControlled.getConfigurator().apply(climbConfig.climbRotateControlledConfig);
        climbRotateMotorFollower.getConfigurator().apply(climbConfig.climbDeployConfig);
        climbDeploymentMotor.getConfigurator().apply(climbConfig.climbDeployConfig);
    }
    @Override
    public void setDeploymentPostion(ClimbStates state) {
        climbMotionControlledRequest.Position = state.climbPosition;
        climbRotateMotorControlled.setControl(climbMotionControlledRequest);
        climbRotateMotorFollower.setControl(new Follower(ClimbConstants.climbRotateControlledID, MotorAlignmentValue.Opposed));
    
        
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