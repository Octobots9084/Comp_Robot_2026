package frc.robot.subsystems.Intake;

   /**
   * The premade roller speed and intake positions for {@link frc.robot.subsystems.Intake.Intake Intake}
   * <br></br>
   * <b>Roller RPS and Position</b>
   * <ul>
   * <li>INTAKING - 45 - 0.34</li>
   * <li>EXTENDED - 0 - 0.34</li>
   * <li>PARTIALEXTENTION - 45 - 0.28</li>
   * <li>SAFE - 0 - 0</li>
   * <li>REVERSEINTAKING - (-30) - 0.3</li>
   * <li>ZERO - 0 - 0</li>
   * <li>ELEPHANTIASISPART2 - 45 - 0.34</li>
   * </ul>
   */
public enum IntakeStates {
    INTAKING(35, 0.30),
    EXTENDED(0, 0.30),
    PARTIALEXTENTION(25,0.15),
    SAFE(0, 0),
    REVERSEINTAKING(-35, 0.30),
    ZERO(0, 0),
    ELEPHANTIASISPART2(25,0.30);

    
   /**
   * The roller speed
   */
    public final double rollerRPS;
    /**
   * The intake position, like inside or outside the robot
   */
    public final double intakePosition;

    private IntakeStates(double rollerRPS, double intakePosition) {

        this.rollerRPS = rollerRPS;
        this.intakePosition = intakePosition;
    }
}

