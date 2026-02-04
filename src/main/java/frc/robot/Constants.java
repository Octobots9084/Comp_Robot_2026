// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static boolean isBlueAlliance = true;
  public static Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
  public static Timer timer = new Timer();
  public static int NUM_LEDS;

    public static final double maxTelopVelocity = 1;//TODO
    public static final double maxTelopAngularVelocity = 1;//TODO
    public static final double leftYDeadband = 0.1;//TODO
    public static final double leftXDeadband = 0.1;//TODO
    public static final double rightXDeadband = 0.15;//TODO
    public static final double maxAngularVelocity = 2;//TODO
    // public static final enum currentMode = 1;//TODO
    public static final double maxVelocity = 2;//TODO


     public static int flyWheelRightID = 0;
    public static int flyWheelLeftID = 0;
    public static int hoodID = 0;
    public static int turretID = 0;
    public static int spindexerID = 0;
    public static int verticalFeederID = 0;
    public static int topRollerID = 0;
   
    public static int lemonDetector = 0;

    public static int intakePivotID = 0;
    public static int intakeRollerID = 19;

    public static int climbRotateControlledID = 0;
    public static int climbRotateFollowerID = 0;
    public static int climbDeployID = 0;

     public static RobotTypes robotType = RobotTypes.ALPHA;
    public static enum RobotTypes {
    // alpha
    ALPHA,

    // beta
    BETA,

    //swervebot
    SWERVE
  }


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

    public static CANBus krakenBus = new CANBus("krakenbus");
}