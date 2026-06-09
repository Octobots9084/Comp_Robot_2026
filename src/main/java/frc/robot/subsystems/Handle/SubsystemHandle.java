package frc.robot.subsystems.Handle;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemHandle<T extends Enum<T>> extends SubsystemBase {

    public T wantedState;
    public T currentState;

    public Class<?> autoLogged;
        
    /**WARNING: SystemLocalTimer works in milliseconds. */
    public SystemLocalTimer timer; 

    public SubsystemHandle(T defaultState) {
        this.wantedState = defaultState;
        this.currentState = defaultState;

        try {
            autoLogged = Class.forName(this.getName() + "IOInputsAutoLogged");
        } catch (ClassNotFoundException e) {
            // TODO Auto-generated catch block
            autoLogged = null;
            e.printStackTrace();
        }

    }

    @Override
    public void periodic() {
        handleStateTransitions();
        applyStates();
        logging();
    }

    public abstract void handleStateTransitions();
    public abstract void applyStates();
    public abstract void logging();


    public class SystemLocalTimer {

        Map<String,Long> deltaTime = new HashMap<String,Long>();
        Map<String,Long> time = new HashMap<String,Long>();
        Map<String,TimeContainer> oscillatingTime = new HashMap<String,TimeContainer>();

        public SystemLocalTimer() {};


        /** Returns the delta time of a timer via said timer's id. Creates timer and returns 0 if no timer is found. */
        public Long timeD(String id) {

            if (!deltaTime.containsKey(id)) {
                deltaTime.put(id, System.currentTimeMillis());
                return 1000l;
            }

            long old = deltaTime.get(id);
            deltaTime.put(id, System.currentTimeMillis());
            
            return System.currentTimeMillis() - old;
        }

        /** Returns the current reading of a timer. Creates a timer at about 0 if one does not exist.*/
        public Long time(String id) {
            
            if (!time.containsKey(id)) time.put(id, System.currentTimeMillis());

            return System.currentTimeMillis() - time.get(id);
        }

        /** Provides a boolean that switches on an oscillator. Starts on false.*/
        public boolean timeO(String id, long period) {
            if (!oscillatingTime.containsKey(id)) oscillatingTime.put(id, new TimeContainer(System.currentTimeMillis(), period));

            TimeContainer container = oscillatingTime.get(id);
            if (container.startTime + (2 * container.switchTime) < System.currentTimeMillis()) {

                container.startTime = System.currentTimeMillis();
                oscillatingTime.put(id, container);
            }

            return container.startTime + container.switchTime > System.currentTimeMillis();
        } 


        public void delete(String id) {
            time.remove(id);
        }

        public void deleteDelta(String id) {
            deltaTime.remove(id);
        }

        public void deleteOscillating(String id) {
            oscillatingTime.remove(id);
        }

        /** WARNING: Deletes all timers. */
        public void clean() {
            time.clear();
            deltaTime.clear();
            oscillatingTime.clear();
        }

        /** A way to store information related to oscillating timers. */
        public class TimeContainer {

            public long startTime;
            public long switchTime;

            public TimeContainer(long startTime, long switchTime) {
                this.startTime = startTime;
                this.switchTime = switchTime;
            }
        }
    }
}
