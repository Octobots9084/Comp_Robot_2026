package frc.robot.subsystems.Handle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public abstract class SubystemIOTalonFXHandle {
    public TelemetryHandle handle = new TelemetryHandle(getClass());
    public Map<String, TalonFX> motorArray = new HashMap<String, TalonFX>();

    public SubystemIOTalonFXHandle() {

    }

    public TalonFX addMotor(String name, int deviceId, TalonFXConfiguration config) {
        TalonFX motor = new TalonFX(deviceId, Constants.krakenBus);
        motor.getConfigurator().apply(config);
        PhoenixUtil.tryUntilOk(5, () -> motor.optimizeBusUtilization(0,1.0));

        motorArray.put(name, motor);
        return motor;
    }

    public TalonFX getMotor(String name) {
        return motorArray.get(name);
    }

    public StatusSignal<Angle> getMotorPos(String name) {
        return getMotor(name).getPosition();
    }

    public StatusSignal<Voltage> getMotorVolts(String name) {
        return getMotor(name).getMotorVoltage();
    }

    public StatusSignal<Current> getMotorStator(String name) {
        return getMotor(name).getStatorCurrent();
    }
    public StatusSignal<AngularVelocity> getMotorVelocity(String name) {
        return getMotor(name).getVelocity();
    }


}
