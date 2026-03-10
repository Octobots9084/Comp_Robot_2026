package frc.robot.subsystems.Climb;

public enum ClimbStates {
    CLIMBEDL1(0.75),
    UNCLIMB(0.8), //same as extended
    EXTENDED(0.8), //TODO SET this to a real value
    LATCHING(0.8), //TODO set this to a real value
    IDLE(0);
    

    double climbPosition;

    private ClimbStates(double climbPosition) {
        this.climbPosition = climbPosition;
    }
}