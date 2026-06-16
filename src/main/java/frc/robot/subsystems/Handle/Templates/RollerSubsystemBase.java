package frc.robot.subsystems.Handle.Templates;

import frc.robot.subsystems.Handle.SubsystemHandle;

public abstract class RollerSubsystemBase<T extends Enum<T>> extends SubsystemHandle<T> {
    public RollerSubsystemBase(T state) {
        super(state);
    }
}
