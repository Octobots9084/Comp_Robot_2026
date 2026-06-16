package frc.robot.subsystems.Handle.Templates;

import frc.robot.subsystems.Handle.SubsystemHandle;

public abstract class PivotSubsystemBase<T extends Enum<T>> extends SubsystemHandle<T> {
    public PivotSubsystemBase(T state) {
        super(state);
    }
}
