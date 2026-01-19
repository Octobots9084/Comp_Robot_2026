package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelIOTalonFX implements FlywheelIO{
    public TalonFX flyWheelLeftMotor;
    public TalonFX flyWheelRightMotor;
    public TalonFX topRollerMotor;
    public ShooterConfigurator shooterConfigs;
    private MotionMagicVelocityVoltage flyWheelRequest;
    private MotionMagicVelocityVoltage topRollerRequest;

    public FlywheelIOTalonFX(){
        shooterConfigs = new ShooterConfigurator();
        flyWheelLeftMotor = new TalonFX(ShooterConstants.flyWheelLeftID,"krakenbus");
        flyWheelRightMotor = new TalonFX(ShooterConstants.flyWheelRightID,"krakenbus");
        topRollerMotor = new TalonFX(ShooterConstants.topRollerID,"krakenbus");

        flyWheelRightMotor.getConfigurator().apply(shooterConfigs.flyWheelRightConfig);
        topRollerMotor.getConfigurator().apply(shooterConfigs.topRollerConfig);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs){
        inputs.FlywheelRPS = flyWheelRightMotor.getVelocity().getValueAsDouble();
        inputs.LeftMotorTemp = flyWheelLeftMotor.getDeviceTemp().getValueAsDouble();
        inputs.RightMotorTemp = flyWheelRightMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setFlyWheelVelocity(double RPS){
        flyWheelRequest.Velocity = RPS;
        flyWheelRightMotor.setControl(flyWheelRequest);
        flyWheelRightMotor.setControl(new Follower(ShooterConstants.flyWheelRightID,MotorAlignmentValue.Opposed));
    }

    @Override
    public AngularVelocity getFlyWheelVelocity(){
        return flyWheelRightMotor.getVelocity().getValue();
    }

    @Override
    public boolean flywheelInTolerance(double tolerance){
        return MathUtil.isNear(flyWheelRequest.getVelocityMeasure().in(Units.RadiansPerSecond), this.getFlyWheelVelocity().in(Units.RadiansPerSecond), tolerance);
    }

     @Override
    public void setTopRollerVelocity(double RPS){
        topRollerRequest.Velocity = RPS;
        topRollerMotor.setControl(flyWheelRequest);
    }

    @Override
    public AngularVelocity getTopRollerVelocity(){
        return topRollerMotor.getVelocity().getValue();
    }

    @Override
    public boolean topRollerInTolerance(double tolerance){
        return MathUtil.isNear(topRollerRequest.getVelocityMeasure().in(Units.RadiansPerSecond), this.getTopRollerVelocity().in(Units.RadiansPerSecond), tolerance);
    }
}
