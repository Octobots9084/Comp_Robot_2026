package frc.robot.subsystems.Shooter.Feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Shooter.ShooterConfigurator;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelStates;
import frc.robot.util.PhoenixUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class FeederIOTalonFX implements FeederIO {
    private final StatusSignal<AngularVelocity> feederVelocity;
    private final StatusSignal<AngularVelocity> spindexerVelocity;

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

        feederVelocity = verticalFeederMotor.getVelocity();
        spindexerVelocity = spindexerMotor.getVelocity();

        PhoenixUtil.tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50,feederVelocity,spindexerVelocity));
        PhoenixUtil.tryUntilOk(5, () -> verticalFeederMotor.optimizeBusUtilization(0,1.0));
        PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.optimizeBusUtilization(0,1.0));

        PhoenixUtil.registerSignals(
            Constants.krakenBus.isNetworkFD(),
            feederVelocity,
            spindexerVelocity);
    }

    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederCurrentState = Feeder.getInstance().getCurrentState();
        inputs.spindexerRPS = getSpindexerVelocity();
        inputs.verticalFeederRPS = getVerticalFeederVelocity();
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
        return spindexerVelocity.getValueAsDouble();
    }

    @Override
    public double getVerticalFeederVelocity() {
        return feederVelocity.getValueAsDouble();
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
