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


     public static int flyWheelRightID = 21;
    public static int flyWheelLeftID = 16;
    public static int hoodID = 13;
    public static int turretID = 17;
    public static int spindexerID = 18;
    public static int verticalFeederID = 15;

    public static double maximumHoodPosition = 0.1157;
    public static double maximumTurretPosition = 0.8; //TODO set this to an actual value so sinjin doesnt cry

    public static double turretGearRatio = (3/1)*(256/222);
    public static double hoodGearRatio = (23/20)*(18/0.9);

    public static double rotateGearRatio = 125;//TODO fix this gear ratio
    public static double deployGearRatio = 1;

    public static double intakePivotGearRatio = (5) *(34/29);
   
    public static int lemonDetector = 0;

    public static int intakePivotID = 20;
    public static int intakeRollerID = 19;

    public static int climbRotateControlledID = 14;
    public static int climbRotateFollowerID = 0;
    public static int climbDeployID = 0;

     public static RobotTypes robotType = RobotTypes.BETA;
    //set to ALPHA later
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