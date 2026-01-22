package frc.robot.subsystems.Shooter.Feeder;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;

public class FeederIOTalonFX implements FeederIO{
    public TalonFX multiFeederMotor;
    public TalonFX singleFeederMotor;
    public ShooterConfigurator shooterConfigs;
    private MotionMagicVelocityVoltage multiFeederRequest;
    private MotionMagicVelocityVoltage singleFeederRequest;

    public FeederIOTalonFX(){
        shooterConfigs = new ShooterConfigurator();
        multiFeederMotor = new TalonFX(ShooterConstants.multiFeedertID,GeneralConstants.krakenBus);
        singleFeederMotor = new TalonFX(ShooterConstants.singleFeederID,GeneralConstants.krakenBus);

        multiFeederMotor.getConfigurator().apply(shooterConfigs.multiFeederConfig);
        singleFeederMotor.getConfigurator().apply(shooterConfigs.singleFeederConfig);
    }

    public void updateInputs(FeederIOInputs inputs){
        inputs.MultiFeederRPS = multiFeederMotor.getVelocity().getValueAsDouble();
        inputs.SingleFeederRPS = singleFeederMotor.getVelocity().getValueAsDouble();
        inputs.MultiFeederMotorTemp = multiFeederMotor.getDeviceTemp().getValueAsDouble();
        inputs.SingleFeederMotorTemp = singleFeederMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setFeederVelocity(double multiFeederRPS, double singleFeederRPS){
        multiFeederRequest.Velocity = multiFeederRPS;
        singleFeederRequest.Velocity = singleFeederRPS;
        multiFeederMotor.setControl(multiFeederRequest);
        singleFeederMotor.setControl(singleFeederRequest);
    }

    @Override
    public double getMultiFeederVelocity(){
        return multiFeederMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getSingleFeederVelocity(){
        return singleFeederMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double[] getFeederVelocity(){
        double[] feederVelocity = {this.getMultiFeederVelocity(),this.getSingleFeederVelocity()};
        return feederVelocity;
    }

    @Override
    public boolean multiFeederInTolerance(double tolerance){
        return MathUtil.isNear(multiFeederRequest.getVelocityMeasure().in(Units.RadiansPerSecond), this.getMultiFeederVelocity(), tolerance);
    }

    @Override
    public boolean singleFeederInTolerance(double tolerance){
        return MathUtil.isNear(singleFeederRequest.getVelocityMeasure().in(Units.RadiansPerSecond), this.getSingleFeederVelocity(), tolerance);
    }


}
