package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class IntakeConfigurator {
    public TalonFXConfiguration intakeRollerConfig;
    public TalonFXConfiguration intakePivotConfig;

    public IntakeConfigurator() {
        intakeRollerConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.intakeRollerGearRatio));
        intakePivotConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.intakePivotGearRatio));

        // intake roller config
        intakeRollerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        intakeRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeRollerConfig.CurrentLimits.StatorCurrentLimit = 80;
        intakeRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        intakeRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // create PID gains
        intakeRollerConfig.Slot0.kP = 0.4;
        intakeRollerConfig.Slot0.kI = 0.0;
        intakeRollerConfig.Slot0.kD = 0.0;
        intakeRollerConfig.Slot0.kA = 0.0;
        intakeRollerConfig.Slot0.kV = 0.14;
        intakeRollerConfig.Slot0.kS = 0.0;
        intakeRollerConfig.Slot0.kG = 0.0;


        // intake pivot right config
        intakePivotConfig.CurrentLimits.SupplyCurrentLimit = 20;
        intakePivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakePivotConfig.CurrentLimits.StatorCurrentLimit = 40;
        intakePivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakePivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // create PID gains
        intakePivotConfig.Slot0.kP = 100;
        intakePivotConfig.Slot0.kI = 0.1;
        intakePivotConfig.Slot0.kD = 0.0;
        intakePivotConfig.Slot0.kA = 0.0;
        intakePivotConfig.Slot0.kV = 0.0;
        intakePivotConfig.Slot0.kS = 0.0;
        intakePivotConfig.Slot0.kG = 0.0;

        intakePivotConfig.MotionMagic.MotionMagicAcceleration = 2;
        intakePivotConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
    }
}