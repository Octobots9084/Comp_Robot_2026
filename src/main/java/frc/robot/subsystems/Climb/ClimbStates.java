package frc.robot.subsystems.Climb;

   /**
   * The premade climb positions for {@link frc.robot.subsystems.Climb.Climb Climb}
   * <br></br>
   * <b>Positions</b>
   * <ul>
   * <li>CLIMBEDL1 - 0.75</li>
   * <li>UNCLIMB - 0.8</li>
   * <li>EXTENDED - 0.8</li>
   * <li>LATCHING - 0.8</li>
   * <li>IDLE - 0</li>
   * </ul>
   */
public enum ClimbStates {
    CLIMBEDL1(0.75),
    UNCLIMB(0.8), //same as extended
    EXTENDED(0.8), //TODO SET this to a real value
    LATCHING(0.8), //TODO set this to a real value
    IDLE(0);
    

   /**
   * The climb position
   */
    double climbPosition;

    private ClimbStates(double climbPosition) {
        this.climbPosition = climbPosition;
    }
}