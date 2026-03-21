package frc.robot;

public class SysTestingExternal {
    public void test() {
        new SysTestingSub();

        Sys.get(SysTestingSub.class).wantedStateIs(SysTestingEnum.A);

        // Sys.get(SysTestingSub.class).
    }
}
