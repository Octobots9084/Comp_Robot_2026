// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

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
  public static Timer timer = new Timer();
  public static int NUM_LEDS;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  public static class ShooterConstants {
    public static int flyWheelRightID = 0;
    public static int flyWheelLeftID = 0;
    public static int hoodID = 0;
    public static int turretID = 0;
    public static int spindexerID = 0;
    public static int verticalFeederID = 0;
    public static int topRollerID = 0;
    public static boolean driverShoot = false;
  }

  public static class IntakeConstants {
    public static int intakePivotID = 0;
    public static int intakeRollerID = 0;
  }

  public static class ClimbConstants {
    public static int climbRotateControlledID = 0;
    public static int climbRotateFollowerID = 0;
    public static int climbDeployID = 0;
  }

  public static class GeneralConstants {
    public static CANBus krakenBus = new CANBus("krakenbus");
  }
}
