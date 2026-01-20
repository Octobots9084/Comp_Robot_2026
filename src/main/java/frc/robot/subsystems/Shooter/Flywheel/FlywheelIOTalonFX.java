package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;
import frc.robot.subsystems.Shooter.Feeder.FeederIO.FeederIOInputs;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelIOTalonFX implements FlywheelIO{
    public TalonFX FlywheelLeftMotor;
    public TalonFX FlywheelRightMotor;
    public ShooterConfigurator shooterConfigs;
    private MotionMagicVelocityVoltage FlywheelRightMotorRequest;
    public FlywheelIOTalonFX() {
         shooterConfigs = new ShooterConfigurator();
        FlywheelLeftMotor = new TalonFX(ShooterConstants.flyWheelRightID,"krakenbus");
        FlywheelRightMotor = new TalonFX(ShooterConstants.flyWheelLeftID,"krakenbus");

        FlywheelRightMotor.getConfigurator().apply(shooterConfigs.flyWheelRightConfig);

    }
      public void updateInputs(FlywheelIOInputs inputs){
        inputs.FlywheelLeftRPS = FlywheelLeftMotor.getVelocity().getValueAsDouble();
        inputs.FlywheelRightRPS = FlywheelRightMotor.getVelocity().getValueAsDouble();
        inputs.FlywheelLeftMotorTemp = FlywheelLeftMotor.getDeviceTemp().getValueAsDouble();
        inputs.FlywheelRightMotorTemp = FlywheelRightMotor.getDeviceTemp().getValueAsDouble();
    }
    
    public void setFlywheelVelocity(double FlywheelRightRPS, double FlywheelLeftRPS){
        FlywheelRightMotorRequest.Velocity = FlywheelLeftRPS;
        FlywheelLeftMotor.setControl(new Follower(ShooterConstants.flyWheelRightID, MotorAlignmentValue.Opposed));
        FlywheelRightMotor.setControl(FlywheelRightMotorRequest);
    }
      public double getRightMotorVelocity(){
        return FlywheelRightMotor.getVelocity().getValueAsDouble();
    }

    public double[] getFlywheelVelocity(){
        double[] FlywheelVelocity = {this.getRightMotorVelocity()};
        return FlywheelVelocity;
    }
    public boolean FlywheelInTolerance(double flywheelTolerance){
        return MathUtil.isNear(FlywheelRightMotorRequest.getVelocityMeasure().in(Units.RadiansPerSecond), this.getRightMotorVelocity(), flywheelTolerance);
    }



}
