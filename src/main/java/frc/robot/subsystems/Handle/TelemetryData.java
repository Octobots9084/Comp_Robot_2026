package frc.robot.subsystems.Handle;

public class TelemetryData {
    public Class<?> subsystem;
    public String name;
    public Object data;
    public final int id;
    public static int lastId = 0;

    public TelemetryData(Class<?> subsystem, String name, Object data) {
        this.data = data;
        this.name = name;
        this.subsystem = subsystem;
        id = lastId += 1;
    }

    public String toString() {
        return subsystem.getName() + " " + name + " " + data.toString(); 
    }
}
