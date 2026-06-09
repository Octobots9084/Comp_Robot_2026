package frc.robot.subsystems;

import java.util.HashMap;

public class HashmapWrapperHandle extends HashMap<String, Object> {
    
    @Override
    public String toString() {
        String s = "";

        for (Entry<String, Object> segment : this.entrySet())
            s += (segment.getKey() + ":" + segment.getValue().toString()) + ", ";

        return s;    
    }
}
