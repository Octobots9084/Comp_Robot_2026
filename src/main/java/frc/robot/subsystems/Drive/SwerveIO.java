package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.drive.TunerConstants.TunerSwerveDrivetrain;


public interface SwerveIO {
    @AutoLog
    class SwerveIOInputs {
        public Pose2d Pose = new Pose2d();
        public ChassisSpeeds Speeds = new ChassisSpeeds();
        public SwerveModuleState[] ModuleStates;
        public SwerveModuleState[] ModuleTargets;
        public SwerveModulePosition[] ModulePositions;
        public Rotation2d RawHeading = new Rotation2d();
        public double Timestamp;
        public double OdometryPeriod;
        public int SuccessfulDaqs;
        public int FailedDaqs;
    }

    default void updateInputs(SwerveIOInputs inputs) {}

    // default void updateModuleInputs(ModuleIOInputs... inputs) {}

    default void registerTelemetryFunction(SwerveIOInputs inputs) {}

    default void setSwerveState(SwerveRequest request) {}

    default void resetToParamaterizedRotation(Rotation2d rotation2d) {}

    default void updateSimState() {}
    
    default double getAbsoluteEncoderPositiosn(int index) {return 0;}

    default void resetRobotTranslation(Translation2d translation2d) {}

    default Rotation3d getRotation3d () {return new Rotation3d();}

    default void zeroGyro() {}



   // @Override
    //default void refreshData() {}

}