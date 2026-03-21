package frc.robot;

import java.util.HashMap;
import java.util.Map;

/**This class is used to access and retrieve subsystem instances.*/

//why dont we just put active instances in robot
public class Sys {
        //bracket hell
    private static Map<Class<? extends SubsystemWrap>,SubsystemWrap<? extends Enum<?>>> instances
         = new HashMap<Class<? extends SubsystemWrap>,SubsystemWrap<? extends Enum<?>>>();

    /**
     * This method gets a subsystem from its class.
     * @param c The class of the subsystem.*/
    public static SubsystemWrap<? extends Enum<?>> get(Class<? extends SubsystemWrap<? extends Enum<?>>> c) {
        return instances.get(c);
    }
    public static void set(Class<? extends SubsystemWrap> clazz, SubsystemWrap<? extends Enum<?>> obj) {
        instances.put(clazz, obj);
    }
}
