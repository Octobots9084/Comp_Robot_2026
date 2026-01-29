// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  // vision
  public static final String frontCameraName = "FrontCamera";
  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private static final double camPitch = Units.degreesToRadians(30.0);
  public static final Transform3d robotToCamFront = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -camPitch, 0));
  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static boolean isBlueAlliance = true;
  public static Timer timer = new Timer();

  public static final double maxTelopVelocity = 1;//TODO
    public static final double maxTelopAngularVelocity = 1;//TODO
    public static final double leftYDeadband = 0.1;//TODO
    public static final double leftXDeadband = 0.1;//TODO
    public static final double rightXDeadband = 0.15;//TODO
    public static final double maxAngularVelocity = 2;//TODO
    // public static final enum currentMode = 1;//TODO
    public static final double maxVelocity = 2;//TODO
    public static Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);

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
