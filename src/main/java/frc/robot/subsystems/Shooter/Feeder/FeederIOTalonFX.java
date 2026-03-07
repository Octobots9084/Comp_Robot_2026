package frc.robot.subsystems.Shooter.Feeder;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Shooter.ShooterConfigurator;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelStates;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;

public class FeederIOTalonFX implements FeederIO {
    public TalonFX spindexerMotor;
    public TalonFX verticalFeederMotor;
    public ShooterConfigurator shooterConfigs;
    private VelocityVoltage spindexerRequest = new VelocityVoltage(0);
    private VelocityVoltage verticalFeederRequest = new VelocityVoltage(0);

    public FeederIOTalonFX() {
        shooterConfigs = new ShooterConfigurator();
        spindexerMotor = new TalonFX(Constants.spindexerID, Constants.krakenBus);
        verticalFeederMotor = new TalonFX(Constants.verticalFeederID, Constants.krakenBus);

        spindexerMotor.getConfigurator().apply(shooterConfigs.spindexerConfig);
        verticalFeederMotor.getConfigurator().apply(shooterConfigs.verticalFeederConfig);
    }

    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederCurrentState = Feeder.getInstance().getCurrentState();
        inputs.spindexerRPS = spindexerMotor.getVelocity().getValueAsDouble();
        inputs.verticalFeederRPS = verticalFeederMotor.getVelocity().getValueAsDouble();
        // inputs.SpindexerCurrent = spindexerMotor.getStatorCurrent().getValueAsDouble();
        // inputs.verticalFeederCurrent = verticalFeederMotor.getStatorCurrent().getValueAsDouble();

    }

    @Override
    public void setFeederVelocity(FeederStates state) {
        spindexerMotor.setControl(spindexerRequest.withVelocity(state.spindexerRPS));
        verticalFeederMotor.setControl(verticalFeederRequest.withVelocity(state.feederRPS));
        // spindexerMotor.setVoltage(state.spindexerRPS);
        // verticalFeederMotor.setVoltage(state.feederRPS);
    }

    @Override
    public double getSpindexerVelocity() {
        return spindexerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getVerticalFeederVelocity() {
        return verticalFeederMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double[] getFeederVelocity() {
        double[] feederVelocity = { this.getSpindexerVelocity(), this.getVerticalFeederVelocity() };
        return feederVelocity;
    }

    @Override
    public boolean spindexerInTolerance(double tolerance) {
        return MathUtil.isNear(spindexerRequest.getVelocityMeasure().in(Units.RadiansPerSecond),
                this.getSpindexerVelocity(), tolerance);
    }

    @Override
    public boolean verticalFeederInTolerance(double tolerance) {
        return MathUtil.isNear(verticalFeederRequest.getVelocityMeasure().in(Units.RadiansPerSecond),
                this.getVerticalFeederVelocity(), tolerance);
    }

}
