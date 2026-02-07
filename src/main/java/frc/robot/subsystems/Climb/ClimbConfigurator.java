package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimbConfigurator {
    //2 motors spin the climb
    public TalonFXConfiguration climbRotateControlledConfig;
    public TalonFXConfiguration climbDeployConfig;
    

    //1 deploy the climb

    public ClimbConfigurator() {
        climbRotateControlledConfig = new TalonFXConfiguration()
                        .withFeedback(new FeedbackConfigs().
                        withSensorToMechanismRatio(Constants.rotateGearRatio));
        climbDeployConfig = new TalonFXConfiguration()
                        .withFeedback(new FeedbackConfigs().
                        withSensorToMechanismRatio(Constants.deployGearRatio));




        //fly wheel right config
        climbRotateControlledConfig.CurrentLimits.SupplyCurrentLimit = 20;
        climbRotateControlledConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climbRotateControlledConfig.CurrentLimits.StatorCurrentLimit = 40;
        climbRotateControlledConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        climbRotateControlledConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbRotateControlledConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // create PID gains
        climbRotateControlledConfig.Slot0.kP = 0.0;
        climbRotateControlledConfig.Slot0.kI = 0.0;
        climbRotateControlledConfig.Slot0.kD = 0.0;
        climbRotateControlledConfig.Slot0.kA = 0.0;
        climbRotateControlledConfig.Slot0.kV = 0.0;
        climbRotateControlledConfig.Slot0.kS = 0.0;
        climbRotateControlledConfig.Slot0.kG = 0.0;

        climbRotateControlledConfig.MotionMagic.MotionMagicAcceleration = 0;
        climbRotateControlledConfig.MotionMagic.MotionMagicJerk = 0;
        climbRotateControlledConfig.MotionMagic.MotionMagicCruiseVelocity = 0;

        //-----------------------------------------------------------------//

         //fly wheel right config
        climbRotateControlledConfig.CurrentLimits.SupplyCurrentLimit = 20;
        climbRotateControlledConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climbRotateControlledConfig.CurrentLimits.StatorCurrentLimit = 40;
        climbRotateControlledConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        climbRotateControlledConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbRotateControlledConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // create PID gains
        climbRotateControlledConfig.Slot0.kP = 0.0;
        climbRotateControlledConfig.Slot0.kI = 0.0;
        climbRotateControlledConfig.Slot0.kD = 0.0;
        climbRotateControlledConfig.Slot0.kA = 0.0;
        climbRotateControlledConfig.Slot0.kV = 0.0;
        climbRotateControlledConfig.Slot0.kS = 0.0;
        climbRotateControlledConfig.Slot0.kG = 0.0;

        climbRotateControlledConfig.MotionMagic.MotionMagicAcceleration = 0;
        climbRotateControlledConfig.MotionMagic.MotionMagicJerk = 0;
        climbRotateControlledConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    }

}
