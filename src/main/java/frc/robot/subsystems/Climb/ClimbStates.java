package frc.robot.subsystems.Climb;

public enum ClimbStates {
    CLIMBEDL3(0.87, 0),
    CLIMBEDL1(0.75, 0),
    IDLE(0, 0),
    ENGAGEDL3(0.25, 0),
    DEPLOYEDL3(0, 0),
    DEPLOYEDL1(0.5, 0),
    ZERO(0, 0);

    double climbPosition;
    double climbDeployPosition;

    private ClimbStates(double climbPosition, double climbDeployPosition) {
        this.climbPosition = climbPosition;
        this.climbDeployPosition = climbDeployPosition;
    }
}