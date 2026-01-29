package frc.robot.subsystems.Climb;

public enum ClimbStates {
    CLIMBING(0,0),
    IDLE(0,0),
    GRABBING(0,0),
    DEPLOYED(0,0); // 0, more than 0

    double climbPosition;
    double climbDeployPosition;

    private ClimbStates(double climbPosition, double climbDeployPosition) {
        this.climbPosition = climbPosition;
        this.climbDeployPosition = climbDeployPosition;
    }
}