package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX; // I think this is important...
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Revolutions;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;

public class ClimbIOTalonFX implements ClimbIO {
    public DigitalInput zeroingSwitch = new DigitalInput(8);// todo hehe Hello Oliver
    // controlls the climb motor rotate (follower is influenced by extreiror
    // varible)
    public TalonFX climbRotateMotorControlled;
    public TalonFX climbRotateMotorFollower;

    public ClimbConfigurator climbConfig;

    // create two MotionMagicVoltage variables for each of the controlled motors,
    // the followed
    private MotionMagicVoltage climbMotionControlledRequest;

    public ClimbIOTalonFX() {
        climbConfig = new ClimbConfigurator();

        climbRotateMotorControlled = new TalonFX(Constants.climbRotateControlledID, Constants.krakenBus);

        climbRotateMotorControlled.getConfigurator().apply(climbConfig.climbRotateControlledConfig);
        climbMotionControlledRequest = new MotionMagicVoltage(0.0);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbMotorControlledTemperature = climbRotateMotorControlled.getDeviceTemp().getValueAsDouble();
        inputs.climbPosition = climbRotateMotorControlled.getPosition().getValueAsDouble();
    }

    @Override
    public void setClimbState(ClimbStates state) {
        climbMotionControlledRequest.Position = state.climbPosition;
        climbRotateMotorControlled.setControl(climbMotionControlledRequest);
    }

    @Override
    public double getClimbPosition() {
        return climbRotateMotorControlled.getPosition().getValueAsDouble();
    }

    @Override
    public boolean climbInTolerance(double climbTolerance) {
        return MathUtil.isNear(climbMotionControlledRequest.getPositionMeasure().in(Revolutions),
                this.getClimbPosition(), climbTolerance);
    }

    @Override
    public void setRotateVoltage(double voltage) {
        this.climbRotateMotorControlled.setVoltage(voltage);
        this.climbRotateMotorFollower.setVoltage(-voltage);
    }

    @Override
    public void setCurrentLimit(double current) {
        climbConfig.climbRotateControlledConfig.CurrentLimits.StatorCurrentLimit = current;
        this.climbRotateMotorControlled.getConfigurator().apply(climbConfig.climbRotateControlledConfig);
    }

    @Override
    public boolean isAtCurrentLimit() {
        return this.climbRotateMotorControlled.getFault_StatorCurrLimit().getValue();
    }
}