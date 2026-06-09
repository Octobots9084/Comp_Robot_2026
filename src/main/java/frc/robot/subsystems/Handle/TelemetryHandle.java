package frc.robot.subsystems.Handle;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.AutoLog;

public class TelemetryHandle {
    public Map<String, Object> telemetry = new HashMap<>();
    public static Map<Class<?>, TelemetryHandle> handles = new HashMap<>();
    public static SubsystemInputs INSTANCE = new SubsystemInputs();

    public TelemetryHandle(Class<?> c) {
        handles.put(c, this);
    }

    public static void update() {
        handles.forEach((clazz, handle) -> {
            handle.telemetry.forEach((name, data) -> {
                INSTANCE.results.add(clazz.getSimpleName() + " " + name + " " + data.toString());
            });
        });
    }

    public void log(String name, Object data) {
        telemetry.put(name, data);
    }

    @AutoLog
    public static class SubsystemInputs {
        public List<String> results = new ArrayList<>();

        public void setInputs(List<String> results) {this.results = results;}
    }
}
