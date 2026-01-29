package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;

public class FlywheelIOTalonFX implements FlywheelIO{
    // public TalonFX FlywheelLeftMotor;
    // public TalonFX FlywheelRightMotor;
    public ShooterConfigurator shooterConfigs;
    private MotionMagicVelocityVoltage FlywheelRightMotorRequest;
    public FlywheelIOTalonFX() {
         shooterConfigs = new ShooterConfigurator();
        // FlywheelLeftMotor = new TalonFX(ShooterConstants.flyWheelRightID,GeneralConstants.krakenBus);
        // FlywheelRightMotor = new TalonFX(ShooterConstants.flyWheelLeftID,GeneralConstants.krakenBus);

        // FlywheelRightMotor.getConfigurator().apply(shooterConfigs.flyWheelRightConfig);
    }
    @Override
      public void updateInputs(FlywheelIOInputs inputs){
        // inputs.FlywheelLeftRPS = FlywheelLeftMotor.getVelocity().getValueAsDouble();
        // inputs.FlywheelRightRPS = FlywheelRightMotor.getVelocity().getValueAsDouble();
        // inputs.FlywheelLeftMotorTemp = FlywheelLeftMotor.getDeviceTemp().getValueAsDouble();
        // inputs.FlywheelRightMotorTemp = FlywheelRightMotor.getDeviceTemp().getValueAsDouble();
    }
    
    public void setFlywheelVelocity(FlywheelStates state){
        FlywheelRightMotorRequest.Velocity = state.FlywheelRightRPS;
        // FlywheelLeftMotor.setControl(new Follower(ShooterConstants.flyWheelRightID, MotorAlignmentValue.Opposed));
        // FlywheelRightMotor.setControl(FlywheelRightMotorRequest);
    }
    public double getLeftMotorVelocity(){
        return 10; //FlywheelLeftMotor.getVelocity().getValueAsDouble(); value of 10 used for testing porposes
    }
    public double getRightMotorVelocity(){
        return 10; //FlywheelRightMotor.getVelocity().getValueAsDouble(); value of 10 used for testing porposes
    }

    public double[] getFlywheelVelocity(){
        double[] FlywheelVelocity = {this.getRightMotorVelocity(),this.getLeftMotorVelocity()};
        return FlywheelVelocity;
    }
}
