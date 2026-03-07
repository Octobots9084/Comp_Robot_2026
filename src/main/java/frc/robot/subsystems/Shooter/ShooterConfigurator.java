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

    public ShooterConfigurator() {
        flyWheelRightConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((1)/((Constants.flywheelRadius*Constants.flywheelGearRatio+Constants.topRollerRadius*Constants.flywheelGearRatio*Constants.flywheelToTopRollerRatio)*Math.PI)));
        turretConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.turretGearRatio));
        hoodConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.hoodGearRatio));
        spindexerConfig = new TalonFXConfiguration();
        verticalFeederConfig = new TalonFXConfiguration();
        // fly wheel right config
        flyWheelRightConfig.CurrentLimits.SupplyCurrentLimit = 20;
        flyWheelRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flyWheelRightConfig.CurrentLimits.StatorCurrentLimit = 60;
        flyWheelRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        flyWheelRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flyWheelRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // create PID gains
        flyWheelRightConfig.Slot0.kP = 2.5;
        flyWheelRightConfig.Slot0.kI = 0.0;
        flyWheelRightConfig.Slot0.kD = 0.0;
        flyWheelRightConfig.Slot0.kA = 0.0;
        flyWheelRightConfig.Slot0.kV = 0.6;
        flyWheelRightConfig.Slot0.kS = 0.0;
        flyWheelRightConfig.Slot0.kG = 0.0;

        flyWheelRightConfig.MotionMagic.MotionMagicAcceleration = 100;
        flyWheelRightConfig.MotionMagic.MotionMagicJerk = 100;
        flyWheelRightConfig.MotionMagic.MotionMagicCruiseVelocity = 1000;

        
        // turret config
        turretConfig.CurrentLimits.SupplyCurrentLimit = 20;
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.CurrentLimits.StatorCurrentLimit = 40;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // create PID gains
        turretConfig.Slot0.kP = 1125; //1125;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 0.0;
        turretConfig.Slot0.kA = 0.0;
        turretConfig.Slot0.kV = 0.0;
        turretConfig.Slot0.kS = 0; //0.5;
        turretConfig.Slot0.kG = 0.0;

        turretConfig.MotionMagic.MotionMagicAcceleration = 10;
        turretConfig.MotionMagic.MotionMagicJerk = 100;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 2.75;//TODO lower this to reduce overshoot when wrapping

        // hood config
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 20;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 20;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // create PID gains
        hoodConfig.Slot0.kP = 125;//5;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;
        hoodConfig.Slot0.kA = 0.0;
        hoodConfig.Slot0.kV = 0.0;
        hoodConfig.Slot0.kS = 0.0;
        hoodConfig.Slot0.kG = -0.3;

        hoodConfig.MotionMagic.MotionMagicAcceleration = 10;
        hoodConfig.MotionMagic.MotionMagicJerk = 100;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 4;

        // spindexer config(four lane highway)
        spindexerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.StatorCurrentLimit = 40;
        spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // create PID gains
        spindexerConfig.Slot0.kP = 1;
        spindexerConfig.Slot0.kI = 0.0;
        spindexerConfig.Slot0.kD = 0.0;
        spindexerConfig.Slot0.kA = 0.0;
        spindexerConfig.Slot0.kV = 0.0;
        spindexerConfig.Slot0.kS = 0.0;
        spindexerConfig.Slot0.kG = 0.0;

        spindexerConfig.MotionMagic.MotionMagicAcceleration = 0;
        spindexerConfig.MotionMagic.MotionMagicJerk = 0;
        spindexerConfig.MotionMagic.MotionMagicCruiseVelocity = 0;

        // vertical Feeder config(rural road)
        verticalFeederConfig.CurrentLimits.SupplyCurrentLimit = 20;
        verticalFeederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        verticalFeederConfig.CurrentLimits.StatorCurrentLimit = 60;
        verticalFeederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        verticalFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // create PID gains
        verticalFeederConfig.Slot0.kP = 1;
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
