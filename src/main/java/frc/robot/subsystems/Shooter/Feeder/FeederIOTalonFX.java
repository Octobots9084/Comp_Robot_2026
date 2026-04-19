package frc.robot.subsystems.Shooter.Feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Shooter.ShooterConfigurator;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelStates;
import frc.robot.util.PhoenixUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class FeederIOTalonFX implements FeederIO {
    private final StatusSignal<AngularVelocity> feederVelocity;
    private final StatusSignal<AngularVelocity> spindexerVelocity;
    private final StatusSignal<Voltage> feederVoltage;
    private final StatusSignal<Voltage> spindexerVoltage;
    private final StatusSignal<Current> feederCurrent;
    private final StatusSignal<Current> spindexerCurrent;

    public TalonFX spindexerMotor;
    public TalonFX verticalFeederMotor;
    public TalonFX spindexerFollower;
    public ShooterConfigurator shooterConfigs;
    private VelocityVoltage spindexerRequest = new VelocityVoltage(0);
    private VelocityVoltage verticalFeederRequest = new VelocityVoltage(0);
    public boolean upToSpeed = false;
    public double timeToGetToSpeed = 0;
    public double reveseTimer = 0;

    private Follower followSpindexer = new Follower(Constants.spindexerID, MotorAlignmentValue.Aligned);

    public void resetUpToSpeed(){
        this.upToSpeed = false;
        timeToGetToSpeed = Timer.getFPGATimestamp();
    }

    public FeederIOTalonFX() {
        shooterConfigs = new ShooterConfigurator();
        spindexerMotor = new TalonFX(Constants.spindexerID, Constants.krakenBus);
        spindexerFollower = new TalonFX(Constants.spindexerFollowerID, Constants.krakenBus);
        verticalFeederMotor = new TalonFX(Constants.verticalFeederID, Constants.krakenBus);

        spindexerMotor.getConfigurator().apply(shooterConfigs.spindexerConfig);

          spindexerFollower.setControl(followSpindexer);


        verticalFeederMotor.getConfigurator().apply(shooterConfigs.verticalFeederConfig);

        feederVelocity = verticalFeederMotor.getVelocity();
        spindexerVelocity = spindexerMotor.getVelocity();
        feederVoltage = verticalFeederMotor.getMotorVoltage();
        feederCurrent = verticalFeederMotor.getStatorCurrent();
        PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.optimizeBusUtilization(0,1.0));
        spindexerVoltage = spindexerMotor.getMotorVoltage();
        spindexerCurrent = spindexerMotor.getStatorCurrent();

        PhoenixUtil.tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50,feederVelocity,spindexerVelocity));
        PhoenixUtil.tryUntilOk(5, () -> verticalFeederMotor.optimizeBusUtilization(0,1.0));

        PhoenixUtil.registerSignals(
            Constants.krakenBus.isNetworkFD(),
            feederVelocity,
            spindexerVelocity,
            feederVoltage,
            feederCurrent,
            spindexerVoltage,
            spindexerCurrent);
    }

    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederCurrentState = Feeder.getInstance().getCurrentState();
        inputs.spindexerRPS = getSpindexerVelocity();
        inputs.verticalFeederRPS = getVerticalFeederVelocity();
        inputs.wantedSpindexerRPS = spindexerRequest.Velocity;
        inputs.wantedVerticalFeederRPS = verticalFeederRequest.Velocity;
        inputs.spindexerCurrent = spindexerCurrent.getValueAsDouble();
        inputs.feederCurrent = feederCurrent.getValueAsDouble();
        inputs.spindexerVoltage = spindexerVoltage.getValueAsDouble();
        inputs.feederVoltage = feederVoltage.getValueAsDouble();

    }

    @Override
    public void setFeederVelocity(FeederStates state) {
        if (reveseTimer > Timer.getFPGATimestamp()){
            spindexerMotor.setControl(spindexerRequest.withVelocity(FeederStates.UNJAM.feederRPS));
            verticalFeederMotor.setControl(verticalFeederRequest.withVelocity(FeederStates.UNJAM.feederRPS));
        }else{
            spindexerMotor.setControl(spindexerRequest.withVelocity(state.spindexerRPS));
            verticalFeederMotor.setControl(verticalFeederRequest.withVelocity(state.feederRPS));
            if (state == FeederStates.SCORING || state == FeederStates.FIXEDFIRE){
                if ((this.getSpindexerVelocity()< 0.2 && upToSpeed) || (this.getSpindexerVelocity() < 0.2 && Timer.getFPGATimestamp()-timeToGetToSpeed > 3.5
                ) ){
                    reveseTimer = Timer.getFPGATimestamp() + 0.05;
                    upToSpeed = false;
                }
                else if (this.getSpindexerVelocity()>1.0){
                    upToSpeed = true;
                    timeToGetToSpeed = Timer.getFPGATimestamp();
                }
            }
        }
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
