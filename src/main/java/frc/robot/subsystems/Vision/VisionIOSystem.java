package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class VisionIOSystem implements VisionIO {
    private final PhotonCamera frontCamera;
    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;
    // private final PhotonCamera intakeCamera;
    private final PhotonPoseEstimator photonEstimatorFront;
    private final PhotonPoseEstimator photonEstimatorLeft;
    private final PhotonPoseEstimator photonEstimatorRight;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    private double visonCycleTime;

    // // Simulation
    // private PhotonCameraSim cameraSim;
    // private VisionSystemSim visionSim;

    public VisionIOSystem(EstimateConsumer estConsumer) {
        // intakeCamera = new PhotonCamera(Constants.intakeCameraName);
        // intakeCamera.setDriverMode(true);
        // CameraServer.startAutomaticCapture(Constants.intakeCameraName, "/dev/video0");
        frontCamera = new PhotonCamera(Constants.frontCameraName);
        leftCamera = new PhotonCamera(Constants.leftCameraName);
        rightCamera = new PhotonCamera(Constants.rightCameraName);
        photonEstimatorFront = new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamFront);
        photonEstimatorRight = new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamRight);
        photonEstimatorLeft = new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamLeft);
        this.estConsumer = estConsumer; // Lamba that will accept a pose estimate and pass it to your desired {@link
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // inputs.intakeCameraConected = intakeCamera.isConnected();
        inputs.frontCameraConected = frontCamera.isConnected();
        inputs.rightCameraConected = rightCamera.isConnected();
        inputs.leftCameraConected = leftCamera.isConnected();
        inputs.visonCycleTime = this.visonCycleTime;
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : 
            rightCamera.getAllUnreadResults()
        ) {
            visionEst = photonEstimatorRight.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimatorRight.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
        for (var result : 
            leftCamera.getAllUnreadResults()
        ) {
            visionEst = photonEstimatorLeft.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimatorLeft.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
    
        for (var result : 
            frontCamera.getAllUnreadResults()
        ) {
            visionEst = photonEstimatorFront.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimatorFront.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
        this.visonCycleTime = Timer.getFPGATimestamp()-startTime;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimatorFront.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = Constants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
