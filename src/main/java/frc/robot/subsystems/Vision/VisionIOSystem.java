package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class VisionIOSystem implements VisionIO{
    private final PhotonCamera frontCamera;
    private final PhotonCamera intakeCamera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    
    // // Simulation
    // private PhotonCameraSim cameraSim;
    // private VisionSystemSim visionSim;

    public VisionIOSystem(EstimateConsumer estConsumer){
        intakeCamera = new PhotonCamera(Constants.intakeCameraName);
        intakeCamera.setDriverMode(true);
        CameraServer.startAutomaticCapture(Constants.intakeCameraName, "/dev/video0");
        frontCamera = new PhotonCamera(Constants.frontCameraName);
        photonEstimator = new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamFront);
        this.estConsumer = estConsumer; // Lamba that will accept a pose estimate and pass it to your desired {@link
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.intakeCameraConected = intakeCamera.isConnected();
        inputs.frontCameraConected = frontCamera.isConnected();
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : frontCamera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());
            

            // if (Robot.isSimulation()) {
            //     visionEst.ifPresentOrElse(
            //             est ->
            //                     getSimDebugField()
            //                             .getObject("VisionEstimation")
            //                             .setPose(est.estimatedPose.toPose2d()),
            //             () -> {
            //                 getSimDebugField().getObject("VisionEstimation").setPoses();
            //             });
            // }

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        SmartDashboard.putNumber("VisionEstimatedPose_X", est.estimatedPose.toPose2d().getX());
                        SmartDashboard.putNumber("VisionEstimatedPose_Y", est.estimatedPose.toPose2d().getY());
                        SmartDashboard.putNumber("VisionEstimatedTimeStampSeconds", Utils.fpgaToCurrentTime(est.timestampSeconds));
                        
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
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

    @Override
    public ChassisSpeeds allignClimb(Pose2d pose, int stage){ //stage -1 is the first interation of this function stage 0 is climbOptionalPreStartPosition stage 1 is climbPrePosition stage 2 is climbEngagedPosition
        Translation2d climbOptionalPreStartPosition;
        Translation2d climbPrePosition;
        Translation2d climbEngagedPosition;
        double TargetRotationRadians;
        boolean willGoToOptionalPosition = false;

        Translation2d targetPosition;

        double xVelocity;
        double YVelocity;
        double RotVelocity;
        
        if(Constants.isBlueAlliance){
            if(pose.getY() > Constants.fieldCenterY){
                TargetRotationRadians = Constants.climbStartRotationBluePosY;
                climbOptionalPreStartPosition = Constants.climbOptionalPreStartPositionBluePosY;
                climbPrePosition = Constants.climbStartPositionBluePosY;
                climbEngagedPosition = Constants.climbEngagedPositionBluePosY;
                if(pose.getX() < Constants.climbNeedsToGoToOptionalPrePositionXBlue){
                    willGoToOptionalPosition = true;
                }
            } else {
                TargetRotationRadians = Constants.climbStartRotationBlueNegY;
                climbOptionalPreStartPosition = Constants.climbOptionalPreStartPositionBlueNegY;
                climbPrePosition = Constants.climbStartPositionBlueNegY;
                climbEngagedPosition = Constants.climbEngagedPositionBlueNegY;
                if(pose.getX() < Constants.climbNeedsToGoToOptionalPrePositionXBlue){
                    willGoToOptionalPosition = true;
                }
            }
        } else {
            if(pose.getY() > Constants.fieldCenterY){
                TargetRotationRadians = Constants.climbStartRotationRedPosY;
                climbOptionalPreStartPosition = Constants.climbOptionalPreStartPositionRedPosY;
                climbPrePosition = Constants.climbStartPositionRedPosY;
                climbEngagedPosition = Constants.climbEngagedPositionRedPosY;
                if(pose.getX() < Constants.climbNeedsToGoToOptionalPrePositionXRed){
                    willGoToOptionalPosition = true;
                }
            } else {
                TargetRotationRadians = Constants.climbStartRotationRedNegY;
                climbOptionalPreStartPosition = Constants.climbOptionalPreStartPositionRedNegY;
                climbPrePosition = Constants.climbStartPositionRedNegY;
                climbEngagedPosition = Constants.climbEngagedPositionRedNegY;
                if(pose.getX() < Constants.climbNeedsToGoToOptionalPrePositionXRed){
                    willGoToOptionalPosition = true;
                }
            }
        }
        
        double disToFinalWantedPose = Math.sqrt((pose.getY() - climbPrePosition.getY()) * (pose.getY() - climbPrePosition.getY()) + (pose.getX() - climbPrePosition.getX()) * (pose.getY() - climbPrePosition.getY()));

        double approatchspeed = Constants.VisionAllignspeed;
        if (disToFinalWantedPose < Constants.VisionAllignTollerance){ //TODO test these tolerances
            approatchspeed = Constants.VisionAllignspeed / (disToFinalWantedPose * 50);
        }
        else{
            approatchspeed = Constants.VisionAllignspeed;
        }
        
        if (Math.abs(pose.getRotation().getRadians() - TargetRotationRadians) < 0.25){
            RotVelocity = (pose.getRotation().getRadians() - TargetRotationRadians) * 10;
        } else {
            RotVelocity = Constants.VisionAllignRotspeed;
        }

        double disToWantedPose;
        
        //getting x/y velocitys
        if (stage == -1){
            if (willGoToOptionalPosition == true){
                stage = 0;
            } else {
                stage = 1;
            }
        }

        if (stage == 0){
            disToWantedPose = Math.sqrt((pose.getY() - climbOptionalPreStartPosition.getY())*(pose.getY() - climbOptionalPreStartPosition.getY()) + (pose.getX() - climbOptionalPreStartPosition.getX())*(pose.getX() - climbOptionalPreStartPosition.getX()));
            if(disToWantedPose < Constants.VisionSubStatesAllignTollerance){
                stage = 1;
                targetPosition = climbPrePosition;
            } else {
                targetPosition = climbOptionalPreStartPosition;
            }
        } else if (stage == 1){
            disToWantedPose = Math.sqrt((pose.getY() - climbPrePosition.getY())*(pose.getY() - climbPrePosition.getY()) + (pose.getX() - climbPrePosition.getX())*(pose.getX() - climbPrePosition.getX()));
            if(disToWantedPose < Constants.VisionSubStatesAllignTollerance && Math.abs(pose.getRotation().getRadians() - TargetRotationRadians) <Constants.VisionAllignRotationTollerance){
                stage = 1;
                targetPosition = climbPrePosition;
            } else {
                targetPosition = climbEngagedPosition;
            }
            targetPosition = climbPrePosition;
        } else if (stage == 2){
            disToWantedPose = Math.sqrt((pose.getY() - climbEngagedPosition.getY())*(pose.getY() - climbEngagedPosition.getY()) + (pose.getX() - climbEngagedPosition.getX())*(pose.getX() - climbEngagedPosition.getX()));
            if(disToWantedPose < Constants.VisionAllignTollerance){
                return new ChassisSpeeds(0,0,RotVelocity);
            } else {
                targetPosition = climbEngagedPosition;
            }
        }
        else {
            throw new ArithmeticException("climb allign stage:"+stage+" invalid");
        }
            
        


        xVelocity = approatchspeed * ((pose.getX() - targetPosition.getX()) / disToWantedPose);
        YVelocity = approatchspeed * ((pose.getY() - targetPosition.getY()) / disToWantedPose);
        
        return new ChassisSpeeds(xVelocity,YVelocity,RotVelocity);
    }
    
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
