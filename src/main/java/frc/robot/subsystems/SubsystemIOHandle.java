package frc.robot.subsystems;

import java.lang.reflect.Field;


public abstract class SubsystemIOHandle {
    HashmapWrapperHandle handle = new HashmapWrapperHandle();


    // /**Adds the standard values for motors. */
    // public void refresh(String subsystemName) {
    //     handle.clear();
        
    //     for (Field field : Constants.class.getFields()) {
    //         if (!field.canAccess(null)) continue;
    //         if (!field.getName().endsWith("ID")) continue;
    //         if (!field.getName().startsWith(subsystemName)) continue;
    //         if (!field.getType().equals(Integer.class)) continue;

    //         String motorName = field.getName().split("ID")[0].split(subsystemName)[1];

    //         handle.put(motorName + "", 5);
    //     }
    // }


}
