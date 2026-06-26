package frc.robot.subsystems.Handle.Templates;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Handle.SubsystemHandle;

public abstract class OtherSubsystemBase<T extends Enum<T>> extends SubsystemHandle<T> {
    List<MotorRollerBase> rollers = new ArrayList<>();
    List<MotorPivotBase> pivots = new ArrayList<>(); //todo replace with a dedicated pivot base
    String internalName;

    public OtherSubsystemBase(T def, String internalName) {
        super(def);
    }
   
    public void addMotors(MotorRollerBase... add) {
     for (MotorRollerBase individualMotor : add) {
        rollers.add(individualMotor);
     }
    }
    public void addMotors(MotorPivotBase... add) {
     for (MotorPivotBase individualMotor : add) {
        pivots.add(individualMotor);
     }
    }
    public MotorRollerBase getRoller(String name) {
        AtomicReference<MotorRollerBase> foundRoller = new AtomicReference<>(null);
        rollers.forEach(a -> {
            if (a.name.equals(a)) foundRoller.set(a);
        });
        if (foundRoller.get() == null) throw new RuntimeException("No roller by the name of " + name + "!");
        return foundRoller.get();
    }
    public MotorPivotBase getPivot(String name) {
        AtomicReference<MotorPivotBase> foundRoller = new AtomicReference<>(null);
        pivots.forEach(a -> {
            if (a.name.equals(a)) foundRoller.set(a);
        });
        if (foundRoller.get() == null) throw new RuntimeException("No roller by the name of " + name + "!");
        return foundRoller.get();
    }
   
    @Overridable
    @Override
    public void logging() {
        log("currentState", currentState);
        log("wantedState", wantedState);

        rollers.forEach(motor -> {
            String name = motor.name + File.separator;

            log(name + "RollerRPS", motor.getVelocity().getValueAsDouble());
            log(name + "rollerTemp", motor.getDeviceTemp().getValueAsDouble());
            log(name + "rollerCurrent", motor.getStatorCurrent().getValueAsDouble());
            log(name + "rollerVoltage", motor.getMotorVoltage().getValueAsDouble());
        });

        pivots.forEach(motor -> {
            String name = motor.name + File.separator;

            log(name + "RollerRPS", motor.getVelocity().getValueAsDouble());
            log(name + "rollerTemp", motor.getDeviceTemp().getValueAsDouble());
            log(name + "rollerCurrent", motor.getStatorCurrent().getValueAsDouble());
            log(name + "rollerVoltage", motor.getMotorVoltage().getValueAsDouble());
        });

    }
    public final void log(String name, Object data) {
        Logger.recordOutput(internalName + File.separator + name + File.separator, data.toString());
    }

}
