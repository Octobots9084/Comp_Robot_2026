package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ShooterConfigurator {
    public TalonFXConfiguration flyWheelRightConfig;
    public TalonFXConfiguration turretConfig;
    public TalonFXConfiguration hoodConfig;
    public TalonFXConfiguration spindexerConfig;
    public TalonFXConfiguration verticalFeederConfig;
    public TalonFXConfiguration topRollerConfig;
    public ShooterConfigurator(){
        flyWheelRightConfig = new TalonFXConfiguration();
        turretConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().
                        withSensorToMechanismRatio(Constants.turretGearRatio));
        hoodConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().
                        withSensorToMechanismRatio(Constants.hoodGearRatio));
        spindexerConfig = new TalonFXConfiguration();
        verticalFeederConfig = new TalonFXConfiguration();
        topRollerConfig = new TalonFXConfiguration();

        //fly wheel right config
        topRollerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        topRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        topRollerConfig.CurrentLimits.StatorCurrentLimit = 40;
        topRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        topRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // create PID gains
        topRollerConfig.Slot0.kP = 0.0;
        topRollerConfig.Slot0.kI = 0.0;
        topRollerConfig.Slot0.kD = 0.0;
        topRollerConfig.Slot0.kA = 0.0;
        topRollerConfig.Slot0.kV = 0.0;
        topRollerConfig.Slot0.kS = 0.0;
        topRollerConfig.Slot0.kG = 0.0;

        topRollerConfig.MotionMagic.MotionMagicAcceleration = 0;
        topRollerConfig.MotionMagic.MotionMagicJerk = 0;
        topRollerConfig.MotionMagic.MotionMagicCruiseVelocity = 0;


        //fly wheel right config
        flyWheelRightConfig.CurrentLimits.SupplyCurrentLimit = 20;
        flyWheelRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flyWheelRightConfig.CurrentLimits.StatorCurrentLimit = 40;
        flyWheelRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        flyWheelRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flyWheelRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // create PID gains
        flyWheelRightConfig.Slot0.kP = 0.0;
        flyWheelRightConfig.Slot0.kI = 0.0;
        flyWheelRightConfig.Slot0.kD = 0.0;
        flyWheelRightConfig.Slot0.kA = 0.0;
        flyWheelRightConfig.Slot0.kV = 0.0;
        flyWheelRightConfig.Slot0.kS = 0.0;
        flyWheelRightConfig.Slot0.kG = 0.0;

        flyWheelRightConfig.MotionMagic.MotionMagicAcceleration = 0;
        flyWheelRightConfig.MotionMagic.MotionMagicJerk = 0;
        flyWheelRightConfig.MotionMagic.MotionMagicCruiseVelocity = 0;

        

        //turret config
        turretConfig.CurrentLimits.SupplyCurrentLimit = 20;
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.CurrentLimits.StatorCurrentLimit = 40;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // create PID gains
        turretConfig.Slot0.kP = 0.0;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 0.0;
        turretConfig.Slot0.kA = 0.0;
        turretConfig.Slot0.kV = 0.0;
        turretConfig.Slot0.kS = 0.0;
        turretConfig.Slot0.kG = 0.0;

        turretConfig.MotionMagic.MotionMagicAcceleration = 0;
        turretConfig.MotionMagic.MotionMagicJerk = 0;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 0;



        //hood config
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 20;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 40;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // create PID gains
        hoodConfig.Slot0.kP = 0.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;
        hoodConfig.Slot0.kA = 0.0;
        hoodConfig.Slot0.kV = 0.0;
        hoodConfig.Slot0.kS = 0.0;
        hoodConfig.Slot0.kG = 0.0;

        hoodConfig.MotionMagic.MotionMagicAcceleration = 0;
        hoodConfig.MotionMagic.MotionMagicJerk = 0;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 0;


        //spindexer config(four lane highway)
        spindexerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.StatorCurrentLimit = 40;
        spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // create PID gains
        spindexerConfig.Slot0.kP = 0.0;
        spindexerConfig.Slot0.kI = 0.0;
        spindexerConfig.Slot0.kD = 0.0;
        spindexerConfig.Slot0.kA = 0.0;
        spindexerConfig.Slot0.kV = 0.0;
        spindexerConfig.Slot0.kS = 0.0;
        spindexerConfig.Slot0.kG = 0.0;

        spindexerConfig.MotionMagic.MotionMagicAcceleration = 0;
        spindexerConfig.MotionMagic.MotionMagicJerk = 0;
        spindexerConfig.MotionMagic.MotionMagicCruiseVelocity = 0;



        //vertical Feeder config(rural road)
        verticalFeederConfig.CurrentLimits.SupplyCurrentLimit = 20;
        verticalFeederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        verticalFeederConfig.CurrentLimits.StatorCurrentLimit = 40;
        verticalFeederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        verticalFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // create PID gains
        verticalFeederConfig.Slot0.kP = 0.0;
        verticalFeederConfig.Slot0.kI = 0.0;
        verticalFeederConfig.Slot0.kD = 0.0;
        verticalFeederConfig.Slot0.kA = 0.0;
        verticalFeederConfig.Slot0.kV = 0.0;
        verticalFeederConfig.Slot0.kS = 0.0;
        verticalFeederConfig.Slot0.kG = 0.0;

        verticalFeederConfig.MotionMagic.MotionMagicAcceleration = 0;
        verticalFeederConfig.MotionMagic.MotionMagicJerk = 0;
        verticalFeederConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    }
}
