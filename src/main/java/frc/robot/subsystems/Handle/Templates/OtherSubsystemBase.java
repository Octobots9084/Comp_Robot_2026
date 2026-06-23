package frc.robot.subsystems.Handle.Templates;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Handle.SubsystemHandle;

public abstract class OtherSubsystemBase<T extends Enum<T>> extends SubsystemHandle<T> {
    List<MotorRollerBase> rollers = new ArrayList<>();
    List<MotorRollerBase> pivots = new ArrayList<>(); //todo replace with a dedicated pivot base
    String internalName;

    public OtherSubsystemBase(T def, String internalName) {
        super(def);
    }
   
    public void addRollers(MotorRollerBase... add) {
     for (MotorRollerBase individualMotor : add) {
        rollers.add(individualMotor);
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
   
    @Overridable
    @Override
    public void logging() {
    
    }
    public final void log(String name, Object data) {
        Logger.recordOutput(internalName + "/" + name, data.toString());
    }

}
