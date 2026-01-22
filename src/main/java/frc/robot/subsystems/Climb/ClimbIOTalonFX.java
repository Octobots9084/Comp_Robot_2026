package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX; // I think this is important...

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.GeneralConstants;

public class ClimbIOTalonFX {
    //controlls the climb motor rotate (follower is influenced by extreiror varible)
    public TalonFX climbRotateMotorControlled;
    public TalonFX climbRotateMotorFollower;
    //the deployment varible for deployment motor
    public TalonFX climbDeploymentMotor;

    public ClimbConfigurator climbConfig; 

    //create two MotionMagicVoltage variables for each of the controlled motors, the followed 
    private MotionMagicVoltage climbMotionMotorControlled;
    private MotionMagicVoltage climbMotionMotorFollower;
    private MotionMagicVoltage climbMotionMotor;

    public ClimbIOTalonFX() {
        climbConfig = new ClimbConfigurator();
        
        climbRotateMotorControlled = new TalonFX(ClimbConstants.climbRotateControlledID, GeneralConstants.krakenBus);
        climbRotateMotorFollower = new TalonFX(ClimbConstants.climbRotateFollowerID, GeneralConstants.krakenBus);
        climbDeploymentMotor = new TalonFX(ClimbConstants.climbDeployID, GeneralConstants.krakenBus);
        

        climbRotateMotorControlled.getConfigurator().apply(climbConfig.climbRotateControlledConfig);
        climbDeploymentMotor.getConfigurator().apply(climbConfig.climbDeployConfig);
    }

    public void setDeploymentPostion(){


    }
    
    

}
