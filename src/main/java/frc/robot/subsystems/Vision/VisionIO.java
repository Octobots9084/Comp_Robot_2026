package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean cameraConnected = false;

        public Pose2d camPose2d;

        public PhotonPipelineResult camResult;
        
        
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void updatePose(){}

    public default void addVisionReading(String cameraName, EstimatedRobotPose pose, Matrix<N3, N1> visionMeasurementStdDevs){}

    public default void addGyroReading(){}
}
