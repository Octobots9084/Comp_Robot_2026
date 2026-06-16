package frc.robot.subsystems.Handle.Templates;

import frc.robot.subsystems.Handle.SubsystemHandle;

public abstract class PivotWithFollowerSubsystemBase<T extends Enum<T>> extends SubsystemHandle<T> {
    public PivotWithFollowerSubsystemBase(T state) {
        super(state);
    }
}
