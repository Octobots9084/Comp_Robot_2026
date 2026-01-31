// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double maxTelopVelocity = 1;//TODO
    public static final double maxTelopAngularVelocity = 1;//TODO
    public static final double leftYDeadband = 0.5;//TODO
    public static final double leftXDeadband = 0.5;//TODO
    public static final double rightXDeadband = 0.15;//TODO
    public static final double maxAngularVelocity = 2;//TODO
    // public static final enum currentMode = 1;//TODO
    public static final double maxVelocity = 1;//TODO
    public static Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
    public static RobotTypes robotType = RobotTypes.ALPHA;
    public static enum RobotTypes {
    // mango
    ALPHA,

    // other bot
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
}
