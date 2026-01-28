package frc.robot.subsystems.Shooter.Feeder;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;

public class FeederIOTalonFX implements FeederIO{
    public TalonFX spindexerMotor;
    public TalonFX verticalFeederMotor;
    public ShooterConfigurator shooterConfigs;
    private MotionMagicVelocityVoltage spindexerRequest;
    private MotionMagicVelocityVoltage verticalFeederRequest;

    public FeederIOTalonFX(){
        shooterConfigs = new ShooterConfigurator();
        spindexerMotor = new TalonFX(Constants.spindexerID,Constants.krakenBus);
        verticalFeederMotor = new TalonFX(Constants.verticalFeederID,Constants.krakenBus);

        spindexerMotor.getConfigurator().apply(shooterConfigs.spindexerConfig);
        verticalFeederMotor.getConfigurator().apply(shooterConfigs.verticalFeederConfig);
    }

    public void updateInputs(FeederIOInputs inputs){
        inputs.spindexerRPS = spindexerMotor.getVelocity().getValueAsDouble();
        inputs.verticalFeederRPS = verticalFeederMotor.getVelocity().getValueAsDouble();
        inputs.spindexerMotorTemp = spindexerMotor.getDeviceTemp().getValueAsDouble();
        inputs.verticalFeederMotorTemp = verticalFeederMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setFeederVelocity(double spindexerRPS, double verticalFeederRPS){
        spindexerRequest.Velocity = spindexerRPS;
        verticalFeederRequest.Velocity = verticalFeederRPS;
        spindexerMotor.setControl(spindexerRequest);
        verticalFeederMotor.setControl(verticalFeederRequest);
    }

    @Override
    public double getSpindexerVelocity(){
        return spindexerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getVerticalFeederVelocity(){
        return verticalFeederMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double[] getFeederVelocity(){
        double[] feederVelocity = {this.getSpindexerVelocity(),this.getVerticalFeederVelocity()};
        return feederVelocity;
    }

    @Override
    public boolean spindexerInTolerance(double tolerance){
        return MathUtil.isNear(spindexerRequest.getVelocityMeasure().in(Units.RadiansPerSecond), this.getSpindexerVelocity(), tolerance);
    }

    @Override
    public boolean verticalFeederInTolerance(double tolerance){
        return MathUtil.isNear(verticalFeederRequest.getVelocityMeasure().in(Units.RadiansPerSecond), this.getVerticalFeederVelocity(), tolerance);
    }


}
