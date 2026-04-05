package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleBinaryOperator;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonTrackedTargetProto;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class VisionIOSystem implements VisionIO {
    private final PhotonCamera frontLeftCamera;
    private final PhotonCamera frontRightCamera;
    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;
    private final PhotonCamera backCamera;
    // private final PhotonCamera intakeCamera;
    private final PhotonPoseEstimator photonEstimatorFrontRight;
    private final PhotonPoseEstimator photonEstimatorFrontLeft;
    private final PhotonPoseEstimator photonEstimatorLeft;
    private final PhotonPoseEstimator photonEstimatorRight;
    private final PhotonPoseEstimator photonEstimatorBack;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    private double visonCycleTime;
    public static int climbAlignStage = 0;

    public static PIDController xPidcontroller = new PIDController(2,0.2,0.01);
    public static PIDController yPidcontroller = new PIDController(2,0.2,0.01);
    public static PIDController angularPidcontroller = new PIDController(3, 0.5, 0);

    // // Simulation
    // private PhotonCameraSim cameraSim;
    // private VisionSystemSim visionSim;

    public VisionIOSystem(EstimateConsumer estConsumer) {
        // intakeCamera = new PhotonCamera(Constants.intakeCameraName);
        // intakeCamera.setDriverMode(true);
        // CameraServer.startAutomaticCapture(Constants.intakeCameraName, "/dev/video0");
        frontRightCamera = new PhotonCamera(Constants.frontRightCameraName);
        frontLeftCamera = new PhotonCamera(Constants.frontleftCameraName);
        leftCamera = new PhotonCamera(Constants.leftCameraName);
        rightCamera = new PhotonCamera(Constants.rightCameraName);
        backCamera = new PhotonCamera(Constants.backCameraName);
        photonEstimatorFrontRight = new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamFrontRight);
        photonEstimatorFrontLeft = new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamFrontLeft);
        photonEstimatorRight = new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamRight);
        photonEstimatorLeft = new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamLeft);
        photonEstimatorBack = new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamBack);
        this.estConsumer = estConsumer; // Lamba that will accept a pose estimate and pass it to your desired {@link
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // inputs.intakeCameraConected = intakeCamera.isConnected();
        inputs.frontLeftCameraConected = frontLeftCamera.isConnected();
        inputs.frontRightCameraConected = frontRightCamera.isConnected();
        inputs.rightCameraConected = rightCamera.isConnected();
        inputs.leftCameraConected = leftCamera.isConnected();
        inputs.backCameraConected = backCamera.isConnected();

    }

    public boolean CamerasConnected(){
        if(frontLeftCamera.isConnected() && frontRightCamera.isConnected() && rightCamera.isConnected() && leftCamera.isConnected()){
            return true;
        }else{
            return false;
        }
    }
    /*
     * the periodic seaches all cameras for a hub mutitag pose and if found uses only that pose however if it is not found it adds togeter all the other tag poses to get a sutable estimate.
    */
    @Override
    public void periodic() {
        double camConfidance = 0;

        double startTime = Timer.getFPGATimestamp();

        //determines of a mutlitag hub pose was found
        boolean foundSutableMultiTagPoseOnCamBack = false;

        //multitag hub visionmesurment for the right camera
        Optional<EstimatedRobotPose> backHubMultiTagResult = Optional.empty();
        //multitag hub targets for the visionmesurment for the right camera
        Optional<List<PhotonTrackedTarget>> backHubMultiTagTargets = Optional.empty();
        //list of all single tag visionmesurments for the right camera
        ArrayList<Optional<EstimatedRobotPose>> backResults = new ArrayList<Optional<EstimatedRobotPose>>();
        //list of all lists of single tag targets per visionmesurment for the right camera
        ArrayList<List<PhotonTrackedTarget>> backTargets = new ArrayList<List<PhotonTrackedTarget>>();


        //determines of a mutlitag hub pose was found
        boolean foundSutableMultiTagPoseOnCamRight = false;

        //multitag hub visionmesurment for the right camera
        Optional<EstimatedRobotPose> rightHubMultiTagResult = Optional.empty();
        //multitag hub targets for the visionmesurment for the right camera
        Optional<List<PhotonTrackedTarget>> rightHubMultiTagTargets = Optional.empty();
        //list of all single tag visionmesurments for the right camera
        ArrayList<Optional<EstimatedRobotPose>> rightResults = new ArrayList<Optional<EstimatedRobotPose>>();
        //list of all lists of single tag targets per visionmesurment for the right camera
        ArrayList<List<PhotonTrackedTarget>> rightTargets = new ArrayList<List<PhotonTrackedTarget>>();

        //determines of a mutlitag hub pose was found
        boolean foundSutableMultiTagPoseOnCamLeft = false;

        //multitag hub visionmesurment for the right camera
        Optional<EstimatedRobotPose> leftHubMultiTagResult = Optional.empty();
        //multitag hub targets for the visionmesurment for the right camera
        Optional<List<PhotonTrackedTarget>> leftHubMultiTagTargets = Optional.empty();
        //list of all single tag visionmesurments for the right camera
        ArrayList<Optional<EstimatedRobotPose>> leftResults = new ArrayList<Optional<EstimatedRobotPose>>();
        //list of all lists of single tag targets per visionmesurment for the right camera
        ArrayList<List<PhotonTrackedTarget>> leftTargets = new ArrayList<List<PhotonTrackedTarget>>();
        
        //determines of a mutlitag hub pose was found
        boolean foundSutableMultiTagPoseOnCamFrontRight = false;

        //multitag hub visionmesurment for the right camera
        Optional<EstimatedRobotPose> frontRightHubMultiTagResult = Optional.empty();
        //multitag hub targets for the visionmesurment for the right camera
        Optional<List<PhotonTrackedTarget>> frontRightHubMultiTagTargets = Optional.empty();
        //list of all single tag visionmesurments for the right camera
        ArrayList<Optional<EstimatedRobotPose>> frontRightResults = new ArrayList<Optional<EstimatedRobotPose>>();
        //list of all lists of single tag targets per visionmesurment for the right camera
        ArrayList<List<PhotonTrackedTarget>> frontRightTargets = new ArrayList<List<PhotonTrackedTarget>>();

        //determines of a mutlitag hub pose was found
        boolean foundSutableMultiTagPoseOnCamFrontLeft = false;

        //multitag hub visionmesurment for the right camera
        Optional<EstimatedRobotPose> frontLeftHubMultiTagResult = Optional.empty();
        //multitag hub targets for the visionmesurment for the right camera
        Optional<List<PhotonTrackedTarget>> frontLeftHubMultiTagTargets = Optional.empty();
        //list of all single tag visionmesurments for the right camera
        ArrayList<Optional<EstimatedRobotPose>> frontLeftResults = new ArrayList<Optional<EstimatedRobotPose>>();
        //list of all lists of single tag targets per visionmesurment for the right camera
        ArrayList<List<PhotonTrackedTarget>> frontLeftTargets = new ArrayList<List<PhotonTrackedTarget>>();

        //creates an optionavl variable to store the camera vision estemation
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        boolean addedGoodMultiTagReslt = false;
        //loops through all camera results for each caemra and checkes for a hub multitage result if one is found then no other cameras a cheaked
        for (var result : frontLeftCamera.getAllUnreadResults()) {
            if(result.hasTargets()) {
                result.targets = removeAmbigousTargets(result.targets);

                visionEst = photonEstimatorFrontLeft.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = photonEstimatorFrontLeft.estimateLowestAmbiguityPose(result);
                    if(!visionEst.isEmpty()) {
                        if (camConfidance == 0)
                            camConfidance = 1;
                        frontLeftResults.add(visionEst);
                    }
                } else {
                    camConfidance = 2;
                    int numberOfHubTags = 0;
                    for(int i = 0; i < result.getTargets().size(); i++)
                        if(addToHubTagNumber(result, i)){
                            numberOfHubTags+=1;
                        }
                    if (numberOfHubTags>=2){
                        frontLeftHubMultiTagResult = visionEst;
                        frontLeftHubMultiTagTargets = Optional.of(result.getTargets());
                        foundSutableMultiTagPoseOnCamFrontLeft = true;
                        addedGoodMultiTagReslt = true;
                        break;
                    }
                    else{
                        frontLeftResults.add(visionEst);
                    }
                    
                }
                frontLeftTargets.add(result.getTargets());
            }
        }
        if (!addedGoodMultiTagReslt)
            for (var result : frontRightCamera.getAllUnreadResults()) {
            if(result.hasTargets()) {
                result.targets = removeAmbigousTargets(result.targets);

                visionEst = photonEstimatorFrontRight.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = photonEstimatorFrontRight.estimateLowestAmbiguityPose(result);
                    if(!visionEst.isEmpty()) {
                        if (camConfidance == 0)
                            camConfidance = 1;
                        frontRightResults.add(visionEst);
                    }
                } else {
                    camConfidance = 2;
                    int numberOfHubTags = 0;
                    for(int i = 0; i < result.getTargets().size(); i++)
                        if(addToHubTagNumber(result, i)){
                            numberOfHubTags+=1;
                        }
                    if (numberOfHubTags>=2){
                        frontRightHubMultiTagResult = visionEst;
                        frontRightHubMultiTagTargets = Optional.of(result.getTargets());
                        foundSutableMultiTagPoseOnCamFrontRight = true;
                        addedGoodMultiTagReslt = true;
                        break;
                    }
                    else{
                        frontRightResults.add(visionEst);
                    }
                    
                }
                frontRightTargets.add(result.getTargets());
            }
        }
        if (!addedGoodMultiTagReslt)
            for (var result : rightCamera.getAllUnreadResults()) {
            if(result.hasTargets()) {
                result.targets = removeAmbigousTargets(result.targets);

                visionEst = photonEstimatorRight.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = photonEstimatorRight.estimateLowestAmbiguityPose(result);
                    if(!visionEst.isEmpty()) {
                        if (camConfidance == 0)
                            camConfidance = 1;
                        rightResults.add(visionEst);
                    }
                } else {
                    camConfidance = 2;
                    int numberOfHubTags = 0;
                    for(int i = 0; i < result.getTargets().size(); i++)
                        if(addToHubTagNumber(result, i)){
                            numberOfHubTags+=1;
                        }
                    if (numberOfHubTags>=2){
                        rightHubMultiTagResult = visionEst;
                        rightHubMultiTagTargets = Optional.of(result.getTargets());
                        foundSutableMultiTagPoseOnCamRight = true;
                        addedGoodMultiTagReslt = true;
                        break;
                    }
                    else{
                        rightResults.add(visionEst);
                    }
                    
                }
                rightTargets.add(result.getTargets());
            }
        }
        if (!addedGoodMultiTagReslt)
            for (var result : leftCamera.getAllUnreadResults()) {
            if(result.hasTargets()) {
                result.targets = removeAmbigousTargets(result.targets);

                visionEst = photonEstimatorLeft.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = photonEstimatorLeft.estimateLowestAmbiguityPose(result);
                    if(!visionEst.isEmpty()) {
                        if (camConfidance == 0)
                            camConfidance = 1;
                        leftResults.add(visionEst);
                    }
                } else {
                    camConfidance = 2;
                    int numberOfHubTags = 0;
                    for(int i = 0; i < result.getTargets().size(); i++)
                        if(addToHubTagNumber(result, i)){
                            numberOfHubTags+=1;
                        }
                    if (numberOfHubTags>=2){
                        leftHubMultiTagResult = visionEst;
                        leftHubMultiTagTargets = Optional.of(result.getTargets());
                        foundSutableMultiTagPoseOnCamLeft = true;
                        addedGoodMultiTagReslt = true;
                        break;
                    }
                    else{
                        leftResults.add(visionEst);
                    }
                    
                }
                leftTargets.add(result.getTargets());
            }
        }
        if (!addedGoodMultiTagReslt)
            for (var result : backCamera.getAllUnreadResults()) {
            if(result.hasTargets()) {
                result.targets = removeAmbigousTargets(result.targets);

                visionEst = photonEstimatorBack.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = photonEstimatorBack.estimateLowestAmbiguityPose(result);
                    if(!visionEst.isEmpty()) {
                        if (camConfidance == 0)
                            camConfidance = 1;
                        backResults.add(visionEst);
                    }
                } else {
                    camConfidance = 2;
                    int numberOfHubTags = 0;
                    for(int i = 0; i < result.getTargets().size(); i++)
                        if(addToHubTagNumber(result, i)){
                            numberOfHubTags+=1;
                        }
                    if (numberOfHubTags>=2){
                        backHubMultiTagResult = visionEst;
                        backHubMultiTagTargets = Optional.of(result.getTargets());
                        foundSutableMultiTagPoseOnCamBack = true;
                        addedGoodMultiTagReslt = true;
                        break;
                    }
                    else{
                        backResults.add(visionEst);
                    }
                    
                }
                backTargets.add(result.getTargets());
            }
        }

        //loops thouhg all saved camera results and adds them to pose unless the last section found a hub multi tag pose in witch case that is the only pose added
        if (foundSutableMultiTagPoseOnCamRight){
            Logger.recordOutput("useing Cam Right",true);
            Logger.recordOutput("useing Cam Left",false);
            Logger.recordOutput("useing Cam Front Right",false);
            Logger.recordOutput("useing Cam Front Left",false);
            Logger.recordOutput("Using Cam Back", false);
            updateEstimationStdDevs(rightHubMultiTagResult, rightHubMultiTagTargets.get());

            rightHubMultiTagResult.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
        else if (!addedGoodMultiTagReslt && !rightResults.isEmpty()){
            Logger.recordOutput("useing Cam Right",false);
            Logger.recordOutput("useing Cam Left",false);
            Logger.recordOutput("useing Cam Front Right",false);
            Logger.recordOutput("useing Cam Front Left",false);
            Logger.recordOutput("Using Cam Back", false);
            for(int i = 0; i<rightResults.size();i++){
                updateEstimationStdDevs(rightResults.get(i), rightTargets.get(i));

                rightResults.get(i).ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs();
                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        });
            }
        }

        if (foundSutableMultiTagPoseOnCamLeft){
            Logger.recordOutput("useing Cam Right",false);
            Logger.recordOutput("useing Cam Left",true);
            Logger.recordOutput("useing Cam Front Right",false);
            Logger.recordOutput("useing Cam Front Left",false);
            Logger.recordOutput("Using Cam Back", false);
            updateEstimationStdDevs(leftHubMultiTagResult, leftHubMultiTagTargets.get());

            leftHubMultiTagResult.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
        else if (!addedGoodMultiTagReslt && !leftResults.isEmpty()){
            Logger.recordOutput("useing Cam Right",false);
            Logger.recordOutput("useing Cam Left",false);
            Logger.recordOutput("useing Cam Front Right",false);
            Logger.recordOutput("useing Cam Front Left",false);
            Logger.recordOutput("Using Cam Back", false);
            for(int i = 0; i<leftResults.size();i++){
                updateEstimationStdDevs(leftResults.get(i), leftTargets.get(i));

                leftResults.get(i).ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs();
                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        });
            }
        }

        if (foundSutableMultiTagPoseOnCamFrontRight){
            Logger.recordOutput("useing Cam Right",false);
            Logger.recordOutput("useing Cam Left",false);
            Logger.recordOutput("useing Cam Front Right",true);
            Logger.recordOutput("useing Cam Front Left",false);
            Logger.recordOutput("Using Cam Back", false);
            updateEstimationStdDevs(frontRightHubMultiTagResult, frontRightHubMultiTagTargets.get());

            frontRightHubMultiTagResult.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
        else if (!addedGoodMultiTagReslt && !frontRightResults.isEmpty()){
            Logger.recordOutput("useing Cam Right",false);
            Logger.recordOutput("useing Cam Left",false);
            Logger.recordOutput("useing Cam Front Right",false);
            Logger.recordOutput("useing Cam Front Left",false);
            Logger.recordOutput("Using Cam Back", false);
            for(int i = 0; i<frontRightResults.size();i++){
                updateEstimationStdDevs(frontRightResults.get(i), frontRightTargets.get(i));

                frontRightResults.get(i).ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs();
                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        });
            }
        }

        if (foundSutableMultiTagPoseOnCamFrontLeft){
            Logger.recordOutput("useing Cam Right",false);
            Logger.recordOutput("useing Cam Left",false);
            Logger.recordOutput("useing Cam Front Right",false);
            Logger.recordOutput("useing Cam Front Left",true);
            Logger.recordOutput("Using Cam Back", false);
            updateEstimationStdDevs(frontLeftHubMultiTagResult, frontLeftHubMultiTagTargets.get());

            frontLeftHubMultiTagResult.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
        else if (!addedGoodMultiTagReslt  && !frontLeftResults.isEmpty()){
            Logger.recordOutput("useing Cam Right",false);
            Logger.recordOutput("useing Cam Left",false);
            Logger.recordOutput("useing Cam Front Right",false);
            Logger.recordOutput("useing Cam Front Left",false);
            Logger.recordOutput("Using Cam Back", false);
            for(int i = 0; i<frontLeftResults.size();i++){
                updateEstimationStdDevs(frontLeftResults.get(i), frontLeftTargets.get(i));

                frontLeftResults.get(i).ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs();
                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        });
            }
        }
        if (foundSutableMultiTagPoseOnCamBack){
            Logger.recordOutput("useing Cam Right",true);
            Logger.recordOutput("useing Cam Left",false);
            Logger.recordOutput("useing Cam Front Right",false);
            Logger.recordOutput("useing Cam Front Left",false);
            Logger.recordOutput("Using Cam Back", true);
            updateEstimationStdDevs(backHubMultiTagResult, backHubMultiTagTargets.get());

            backHubMultiTagResult.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
        else if (!addedGoodMultiTagReslt && !backResults.isEmpty()){
            Logger.recordOutput("useing Cam Right",false);
            Logger.recordOutput("useing Cam Left",false);
            Logger.recordOutput("useing Cam Front Right",false);
            Logger.recordOutput("useing Cam Front Left",false);
            Logger.recordOutput("Using Cam Back", false);
            for(int i = 0; i<backResults.size();i++){
                updateEstimationStdDevs(backResults.get(i), backTargets.get(i));

                backResults.get(i).ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs();
                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        });
            }
        }
        if (addedGoodMultiTagReslt){
            Logger.recordOutput("cam confidance",3);
        }
        else {
            Logger.recordOutput("cam confidance",camConfidance);
        }
        this.visonCycleTime = Timer.getFPGATimestamp()-startTime;
    }

    private boolean addToHubTagNumber(PhotonPipelineResult result, int i){
        if(
                result.getTargets().get(i).getFiducialId() == 18
                || result.getTargets().get(i).getFiducialId() == 19
                || result.getTargets().get(i).getFiducialId() == 20
                || result.getTargets().get(i).getFiducialId() == 21
                || result.getTargets().get(i).getFiducialId() == 24
                || result.getTargets().get(i).getFiducialId() == 27
                || result.getTargets().get(i).getFiducialId() == 26
                || result.getTargets().get(i).getFiducialId() == 25
                || result.getTargets().get(i).getFiducialId() == 8
                || result.getTargets().get(i).getFiducialId() == 9
                || result.getTargets().get(i).getFiducialId() == 10
                || result.getTargets().get(i).getFiducialId() == 11
                || result.getTargets().get(i).getFiducialId() == 2
                || result.getTargets().get(i).getFiducialId() == 5
                || result.getTargets().get(i).getFiducialId() == 4
                || result.getTargets().get(i).getFiducialId() == 3
            ){
            return true;
        }
        return false;
    }

    private List<PhotonTrackedTarget> removeAmbigousTargets(List<PhotonTrackedTarget> allTargets){
        List<PhotonTrackedTarget> optimizedTargets = new ArrayList<PhotonTrackedTarget>();
        for(var target : allTargets){
            if(target.poseAmbiguity < 0.2){
                optimizedTargets.add(target);
            }
        }
        return optimizedTargets;
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
                var tagPose = photonEstimatorFrontRight.getFieldTags().getTagPose(tgt.getFiducialId());
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

    public static ChassisSpeeds allignClimb(Pose2d pose){ //stage -1 is the first interation of this function stage 0 is climbOptionalPreStartPosition stage 1 is climbPrePosition stage 2 is climbEngagedPosition
        Translation2d climbPrePosition;
        Translation2d climbEngagedPosition;
        double TargetRotationRadians;
        
        Translation2d targetPosition;

        double xVelocity;
        double YVelocity;
        double RotVelocity;
        
        if(Constants.isBlueAlliance){
            if(pose.getY() > Constants.fieldCenterY){
                TargetRotationRadians = Constants.climbStartRotationBluePosY;
                climbPrePosition = Constants.climbStartPositionBluePosY;
                climbEngagedPosition = Constants.climbEngagedPositionBluePosY;
            } else {
                TargetRotationRadians = Constants.climbStartRotationBlueNegY;
                climbPrePosition = Constants.climbStartPositionBlueNegY;
                climbEngagedPosition = Constants.climbEngagedPositionBlueNegY;
            }
        } else {
            if(pose.getY() > Constants.fieldCenterY){
                TargetRotationRadians = Constants.climbStartRotationRedPosY;
                climbPrePosition = Constants.climbStartPositionRedPosY;
                climbEngagedPosition = Constants.climbEngagedPositionRedPosY;
            } else {
                TargetRotationRadians = Constants.climbStartRotationRedNegY;
                climbPrePosition = Constants.climbStartPositionRedNegY;
                climbEngagedPosition = Constants.climbEngagedPositionRedNegY;
            }
        }

        Logger.recordOutput("climbAlignStage",climbAlignStage);

        //getting x/y velocitys
        if (climbAlignStage == 0){
            targetPosition = climbPrePosition;
            double xErr = pose.getX() - targetPosition.getX();
            double yErr = pose.getY() - targetPosition.getY();
            double rotErr = pose.getRotation().getRadians() - TargetRotationRadians;

            if(Math.abs(xErr) < Constants.VisionSubStateAllignTollerance
                && Math.abs(yErr) < Constants.VisionSubStateAllignTollerance
                && Math.abs(rotErr) < Constants.VisionAllignRotationTollerance){
                    climbAlignStage = 1;
            }
        } else if (climbAlignStage == 1){
            targetPosition = climbEngagedPosition;
            double xErr = pose.getX() - targetPosition.getX();
            double yErr = pose.getY() - targetPosition.getY();
            double rotErr = pose.getRotation().getRadians() - TargetRotationRadians;

            if(Math.abs(xErr) < Constants.VisionSubStateAllignTollerance
                && Math.abs(yErr) < Constants.VisionSubStateAllignTollerance
                && Math.abs(rotErr) < Constants.VisionAllignRotationTollerance){
                climbAlignStage = 2;
            }
        }
        else if(climbAlignStage == 2) {
            return new ChassisSpeeds(0,0,0);
        } 
        else {
            throw new ArithmeticException("climb allign stage:"+climbAlignStage+" invalid");
        }

        double disToWantedPose = 0;
        if (climbAlignStage == 0){
            disToWantedPose = getDistBetweenPoints(pose.getTranslation(),climbPrePosition);
        } else if (climbAlignStage == 1) {
            disToWantedPose = getDistBetweenPoints(pose.getTranslation(),climbEngagedPosition);
        }

        double approatchspeed = Constants.VisionAllignspeed;
        if (disToWantedPose < Constants.VisionAllignTollerance){ //TODO test these tolerances
            xVelocity = xPidcontroller.calculate(pose.getX(),targetPosition.getX());
            YVelocity = yPidcontroller.calculate(pose.getY(),targetPosition.getY());
        }
        else{
            approatchspeed = Constants.VisionAllignspeed;
        }
            
        Logger.recordOutput("climbAlignTargetX",targetPosition.getX());
        Logger.recordOutput("climbAlignTargetY",targetPosition.getY());
        Logger.recordOutput("climbAlignTargetRotation",TargetRotationRadians);
        xVelocity = Constants.VisionAllignspeed * (targetPosition.getX() - pose.getX())/getDistBetweenPoints(targetPosition,pose.getTranslation());
        YVelocity = Constants.VisionAllignspeed * (targetPosition.getY() - pose.getY())/getDistBetweenPoints(targetPosition,pose.getTranslation());
        RotVelocity = angularPidcontroller.calculate(pose.getRotation().getRadians(),TargetRotationRadians);
        
        return new ChassisSpeeds(xVelocity, YVelocity, RotVelocity);
    }

    public static double getDistBetweenPoints(Translation2d pose1,Translation2d pose2){
        return Math.sqrt((
                pose1.getY() - pose2.getY())
                * (pose1.getY() - pose2.getY()
            ) + (
                pose1.getX() - pose2.getX())
                * (pose1.getX() - pose2.getX()
            ));
    }

    public boolean isAlligned(Pose2d pose){
        Translation2d climbEngagedPosition;
        double TargetRotationRadians;

        if(Constants.isBlueAlliance){
            if(pose.getY() > Constants.fieldCenterY){
                TargetRotationRadians = Constants.climbStartRotationBluePosY;
                climbEngagedPosition = Constants.climbEngagedPositionBluePosY;
            } else {
                TargetRotationRadians = Constants.climbStartRotationBlueNegY;
                climbEngagedPosition = Constants.climbEngagedPositionBlueNegY;
            }
        } else {
            if(pose.getY() > Constants.fieldCenterY){
                TargetRotationRadians = Constants.climbStartRotationRedPosY;
                climbEngagedPosition = Constants.climbEngagedPositionRedPosY;
            } else {
                TargetRotationRadians = Constants.climbStartRotationRedNegY;
                climbEngagedPosition = Constants.climbEngagedPositionRedNegY;
            }
        }

        return (Math.abs(pose.getRotation().getRadians() - TargetRotationRadians) < Constants.VisionAllignRotationTollerance) && (getDistBetweenPoints(pose.getTranslation(),climbEngagedPosition)<Constants.VisionAllignTollerance);
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
