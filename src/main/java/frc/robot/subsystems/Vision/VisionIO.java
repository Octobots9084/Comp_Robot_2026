package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean intakeCameraConected = false;
        public boolean frontCameraConected = false;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void addVisionReading(String cameraName, EstimatedRobotPose pose, Matrix<N3, N1> visionMeasurementStdDevs){}

    public default void addGyroReading(){}

    public default ChassisSpeeds allignClimb(Pose2d pose, int stage){return new ChassisSpeeds();}

    public default void periodic() {}
}
