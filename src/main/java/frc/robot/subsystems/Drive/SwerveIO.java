package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

        void logState(SwerveDrivetrain.SwerveDriveState state) {
            this.Pose = state.Pose;
            this.RawHeading = state.RawHeading;
            this.ModuleStates = state.ModuleStates;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.Speeds = state.Speeds;
            this.SuccessfulDaqs = state.SuccessfulDaqs;
            this.FailedDaqs = state.FailedDaqs;
            this.OdometryPeriod = state.OdometryPeriod;
            this.Timestamp = state.OdometryPeriod;
            // RobotState.getInstance().addPoseObservation(new RobotState.SwerveDriveObservation(this.Pose, this.Speeds));
        }
    }
    default void updateSwerveInputs(SwerveIOInputs inputs) {}

    // default void updateModuleInputs(ModuleIOInputs... inputs) {}

    default void registerTelemetryFunction(SwerveIOInputs inputs) {}

    default void setSwerveState(SwerveRequest request) {}

    default void resetRotation() {}

    default void resetToParamaterizedRotation(Rotation2d rotation2d) {}

    default void updateSimState() {}
    
    default double getAbsoluteEncoderPositiosn(int index) {return 0;}

    default void resetRobotTranslation(Translation2d translation2d) {}

   // @Override
    //default void refreshData() {}

}