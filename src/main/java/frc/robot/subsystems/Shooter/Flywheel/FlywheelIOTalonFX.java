package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;
import frc.robot.util.PhoenixUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final StatusSignal<AngularVelocity> flywheelVelocity;

    public static TalonFX FlywheelLeftMotor;
    public static TalonFX FlywheelRightMotor;
    public ShooterConfigurator shooterConfigs;
    private VelocityVoltage FlywheelRightMotorRequest = new VelocityVoltage(0);

    private Follower follow = new Follower(Constants.flyWheelRightID, MotorAlignmentValue.Aligned);

    
    public FlywheelIOTalonFX() {
        shooterConfigs = new ShooterConfigurator();
        FlywheelLeftMotor = new TalonFX(Constants.flyWheelLeftID, Constants.krakenBus);
        FlywheelRightMotor = new TalonFX(Constants.flyWheelRightID, Constants.krakenBus);

        FlywheelRightMotor.getConfigurator().apply(shooterConfigs.flyWheelRightConfig);

        FlywheelLeftMotor.setControl(follow);

        flywheelVelocity = FlywheelRightMotor.getVelocity();

        PhoenixUtil.tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50,flywheelVelocity));
        PhoenixUtil.tryUntilOk(5, () -> FlywheelRightMotor.optimizeBusUtilization(0,1.0));

        PhoenixUtil.registerSignals(
            Constants.krakenBus.isNetworkFD(),
            flywheelVelocity);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.flywheelCurrentState = Flywheel.getInstance().getCurrentState();
        inputs.FlywheelRightRPS = getRightMotorVelocity();
        // inputs.FlywheelLeftCurrent = FlywheelLeftMotor.getStatorCurrent().getValueAsDouble();
        // inputs.FlywheelRightCurrent = FlywheelRightMotor.getStatorCurrent().getValueAsDouble();
        inputs.flywheelWantedSpeed = FlywheelRightMotorRequest.getVelocityMeasure().in(Units.RevolutionsPerSecond);
    }

    @Override
    public void setFlywheelVelocity(FlywheelStates state) {
        Logger.recordOutput("flywheelWantedSpeed", state.FlywheelRightRPS);
        FlywheelRightMotor.setControl(FlywheelRightMotorRequest.withVelocity(state.FlywheelRightRPS));
    }
    @Override
    public void setFlywheelVelocity(double rps) {
        Logger.recordOutput("flywheelWantedSpeed", rps);
        FlywheelRightMotor.setControl(FlywheelRightMotorRequest.withVelocity(rps));
    }

    public double getRightMotorVelocity() {
        return flywheelVelocity.getValueAsDouble();
    }

    @Override
    public boolean FlywheelInTolerance(double tolerance){
        double requestedFlywheelVelocity = FlywheelRightMotorRequest.getVelocityMeasure().in(Units.RevolutionsPerSecond);
        return MathUtil.isNear(requestedFlywheelVelocity, FlywheelRightMotor.getVelocity().getValueAsDouble(),tolerance)&& requestedFlywheelVelocity > 0;
    }

}
