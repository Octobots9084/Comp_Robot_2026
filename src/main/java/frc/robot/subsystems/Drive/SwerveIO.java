package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.subsystems.Drive.BetaConstants.TunerSwerveDrivetrain;
public interface SwerveIO {
    
    @AutoLog
    class SwerveIOInputs {
        public ChassisSpeeds Speeds = new ChassisSpeeds();
        public SwerveModuleState[] ModuleStates;
        public SwerveModuleState[] ModuleTargets;
        public SwerveModulePosition[] ModulePositions;
        public Rotation2d RawHeading = new Rotation2d();
        public double Timestamp;
        public double OdometryPeriod;
        public int SuccessfulDaqs;
        public int FailedDaqs;
        public Pose2d robotPose;
        public Rotation3d GyroRotation;
        public double GyroRoll;
        public double GyroPitch;
        public double GyroYaw;
        public double steerCurrent0;
        public double steerCurrent1;
        public double steerCurrent2;
        public double steerCurrent3;
        public double driveCurrent0;
        public double driveCurrent1;
        public double driveCurrent2;
        public double driveCurrent3;
    }

    default void updateInputs(SwerveIOInputs inputs) {}

    // default void updateModuleInputs(ModuleIOInputs... inputs) {}

    default void registerTelemetryFunction(SwerveIOInputs inputs) {}

    default void setSwerveState(SwerveRequest request) {}

    default void resetToParamaterizedRotation(Rotation2d rotation2d) {}

    default void updateSimState() {}
    
    default double getAbsoluteEncoderPositions(int index) {return 0;}

    default void resetRobotTranslation(Translation2d translation2d) {}

    default ChassisSpeeds getChassisSpeeds() {return new ChassisSpeeds();}

    default Pose2d getPose2d() {return new Pose2d();}

    default SwerveModule[] getSwerveModules(){return new SwerveModule[4];}

    default Rotation2d getGyroYaw() {return new Rotation2d();}

    default SwerveModulePosition[] getModulePositions() {return new SwerveModulePosition[4];}

    default Rotation3d getRotation3d () {return new Rotation3d();}

    default void zeroGyro() {}

    public default double getGyro() {return 0;}

    public default void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {}

    default void setAllianceColor () {}
}