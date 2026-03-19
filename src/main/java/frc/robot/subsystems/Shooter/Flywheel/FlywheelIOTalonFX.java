package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;

public class FlywheelIOTalonFX implements FlywheelIO {
    public static TalonFX FlywheelLeftMotor;
    public static TalonFX FlywheelRightMotor;
    public ShooterConfigurator shooterConfigs;
    private MotionMagicVelocityVoltage FlywheelRightMotorRequest = new MotionMagicVelocityVoltage(0)
            .withAcceleration(100).withSlot(0);

    private Follower follow = new Follower(Constants.flyWheelRightID, MotorAlignmentValue.Opposed);

    
    public FlywheelIOTalonFX() {
        shooterConfigs = new ShooterConfigurator();
        FlywheelLeftMotor = new TalonFX(Constants.flyWheelLeftID, Constants.krakenBus);
        FlywheelRightMotor = new TalonFX(Constants.flyWheelRightID, Constants.krakenBus);

        FlywheelRightMotor.getConfigurator().apply(shooterConfigs.flyWheelRightConfig);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.flywheelCurrentState = Flywheel.getInstance().getCurrentState();
        inputs.FlywheelLeftRPS = FlywheelLeftMotor.getVelocity().getValueAsDouble();
        inputs.FlywheelRightRPS = FlywheelRightMotor.getVelocity().getValueAsDouble();
        // inputs.FlywheelLeftCurrent = FlywheelLeftMotor.getStatorCurrent().getValueAsDouble();
        // inputs.FlywheelRightCurrent = FlywheelRightMotor.getStatorCurrent().getValueAsDouble();
        inputs.flywheelWantedSpeed = FlywheelRightMotorRequest.getVelocityMeasure().in(Units.RevolutionsPerSecond);
    }

    @Override
    public void setFlywheelVelocity(FlywheelStates state) {
        Logger.recordOutput("flywheelWantedSpeed_", state.FlywheelRightRPS);
        FlywheelRightMotor.setControl(FlywheelRightMotorRequest.withVelocity(state.FlywheelRightRPS));
        FlywheelLeftMotor.setControl(follow);
    }
    @Override
    public void setFlywheelVelocity(double rps) {
        Logger.recordOutput("flywheelWantedSpeed_", rps);
        FlywheelRightMotor.setControl(FlywheelRightMotorRequest.withVelocity(rps));
        FlywheelLeftMotor.setControl(follow);
    }

    public double getRightMotorVelocity() {
        return FlywheelRightMotor.getVelocity().getValueAsDouble();
    }

    public double[] getFlywheelVelocity() {
        double[] FlywheelVelocity = { this.getRightMotorVelocity(), this.getLeftMotorVelocity() };

        return FlywheelVelocity;
    }
    @Override
    public boolean FlywheelInTolerance(double tolerance){
        return MathUtil.isNear(FlywheelRightMotorRequest.getVelocityMeasure().in(Units.RevolutionsPerSecond), FlywheelRightMotor.getVelocity().getValueAsDouble(),tolerance);
    }

}
