package frc.robot.subsystems.Climb;

public enum ClimbStates {
    CLIMBING(0),
    IDLE(0),
    GRABBING(0),
    DEPLOYED(0);

    double climbPosition;

    private ClimbStates(double climbPosition) {
        this.climbPosition = climbPosition;
    }
}