package frc.robot.subsystems.Vision;

import java.lang.constant.Constable;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriverCommunications;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Intake;

public class MichaelPieceVision {
    private double inchesToMetersRatio = 0.0254;
    private Transform3d intakeCameraPosition;
    private final Translation3d testRobotPos = new Translation3d(2.2,3,0.08);

    private PhotonCamera camera;
    public List<PhotonTrackedTarget> targets;
    private double yawRotation;
    private double xTransform;
    private double IFOV = (Math.PI/2)/180;//make right
    private double halfFuel = 413/2; //TODO make right

    private int fieldMaxX = 17;//TODO: make real nums, round up
    private int fieldMaxY = 8;
    
    public Translation3d[] poses;
    Translation3d best;
    public Map<Translation3d, Double> values = new HashMap<Translation3d, Double>();
    public StructArrayPublisher<Translation3d> fuelLog = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Fuels", Translation3d.struct).publish();
    public StructPublisher<Pose3d> bestPlaceToGo = NetworkTableInstance.getDefault()
    .getStructTopic("bestPlaceToGo", Pose3d.struct).publish();

    public StructPublisher<Translation3d> fakeRobot3d = NetworkTableInstance.getDefault().getStructTopic("fakeRobot3d", Translation3d.struct).publish();


    private int numTargets = 0;

    
    public MichaelPieceVision(String photonCameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(photonCameraName);
        yawRotation = robotToCamera.getRotation().getZ();
        xTransform = Math.abs(robotToCamera.getTranslation().getX());
        this.intakeCameraPosition = robotToCamera;
        Constants.centerToCameraDefaultPosition = robotToCamera;
    }


    public void updateIntakeCameraPosition () {
        intakeCameraPosition = Constants.centerToCameraDefaultPosition.plus(new Transform3d(Intake.getInstance().io.getIntakePosition()*inchesToMetersRatio,0,0,new Rotation3d()));
    }

    public void updateYawAndX () {
        updateIntakeCameraPosition();
        yawRotation = intakeCameraPosition.getRotation().getZ();
        xTransform = Math.abs(intakeCameraPosition.getTranslation().getX());
    }


    public double getFuelDepthCameraRelative(PhotonTrackedTarget target) {
        List<TargetCorner> corners = target.getDetectedCorners();
        
        if (corners.size() < 2) {
            // SmartDashboard.putNumber("algae depth", -1);
            return -1;
        }
    
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
    
        for (TargetCorner corner : corners) {
            double xAngle = corner.x * IFOV;
            minX = Math.min(minX, xAngle);
            maxX = Math.max(maxX, xAngle);
        }
    
        double angularWidth = maxX - minX;//maybe need to make radians
        double depth = halfFuel / Math.tan(angularWidth / 2); 
    
        // SmartDashboard.putNumber("algae depth", depth);
        return depth;
    }

    public double calculateRobotRelativeYaw(PhotonTrackedTarget target){
        //Positive is the far side of the camera and negative is the close side
        double oppositeSide =  getFuelDepthCameraRelative(target)*Math.cos(yawRotation - target.getYaw()) -xTransform; //was yaw
        double adjacentSide = getFuelDepthCameraRelative(target)*Math.sin(yawRotation - target.getYaw()); // was yaw
        return (Math.PI/2)-Math.atan2(oppositeSide,adjacentSide);
    }

    public double[] getCenterOffsets(){
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()){
            targets = result.getTargets();
            // return calculateRobotRelativeYaw(target);
            double[] yaws = new double[targets.size()];
            for (int i = 0; i < targets.size(); i++) {
                yaws[i] = targets.get(i).getYaw();
            }
            return yaws;
        }
        return null;
    }

    public double[] getOffsetPitches() {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()){
            targets = result.getTargets();
            // return calculateRobotRelativeYaw(target);
            double[] pitches = new double[targets.size()];
            for (int i = 0; i < targets.size(); i++) {
                pitches[i] = targets.get(i).getPitch() + 12.5;//why 12.5?
            }
            return pitches;
        }
        return null;
    }

    public boolean hasTargets(){
        return camera.getLatestResult().hasTargets();
    }

    public Translation3d get3dPoseFieldRelative (PhotonTrackedTarget target) {

        updateIntakeCameraPosition();

        double depth = getFuelDepthCameraRelative(target);

        double yaw = Math.toRadians(target.getYaw());
        double pitch = Math.toRadians(target.getPitch());

        return SwerveSubsystem.getInstance().getRobotPose3d()
            .plus(intakeCameraPosition)
            .plus(
                new Transform3d(new Translation3d(
                    depth * Math.cos(pitch) * Math.cos(yaw), 
                    depth * Math.cos(pitch) * Math.sin(yaw), 
                    depth * Math.sin(pitch)
                ),
                new Rotation3d()))
            .getTranslation();

        // return new Translation3d(x,y,depth)
        //     .rotateBy(robotToCamera.getRotation())
        //     .plus(robotToCamera.getTranslation())
        //     .rotateBy(SwerveSubsystem.getInstance().getRobotPose3d().getRotation())
        //     .plus(SwerveSubsystem.getInstance().getRobotPose3d().getTranslation());
    }

    public void cycle () {
        // findTargets();
        // addTargets();
        // logPoses();
    }

    public void findTargets () {
        targets = camera.getLatestResult().getTargets();
    }

    public void addTargets () {
        if (targets == null) {
            poses = new Translation3d[0];
            return;
        }
        numTargets = targets.size();//targets.size
        poses = new Translation3d[numTargets];

        for (int i = 0; i < targets.size(); i++) {
            //replace w/ real pose
            //poses.add(targets.get(i).something);
            poses[i] = get3dPoseFieldRelative(targets.get(i));
        }
        // poses[0] = new Translation3d(1.5,1.5, 0.08);
    }

    public void setTargetDistances () {
        values.clear();
        for (int i = 0; i < targets.size(); i++) {
            //replace w/ real pose
            //poses.add(targets.get(i).something);
            values.put(poses[i], getFuelDepthCameraRelative(targets.get(i)));
        }

    }

    //do 5x5 around robot? 
    public int[] getRegion (Translation3d target) {
        return new int[]{(int) target.getX(), (int) target.getY()};
    }

    public int numFuelsInRegion (int[] region) {
        int numFuel = 0;
        if (poses.length == 0) {return 0;}
        for (int i = 0; i < poses.length; i++) {
            if ((int) poses[i].getX() == region[0] && (int) poses[i].getY() == region[1]) {
                numFuel++;
            }
        }
        return numFuel;
    }

    public Translation3d bestPlaceToGo () {
        int searchRadius = 2;//radius outside of robot, 1 = 3 diameter, 2 = 5;
        // int[] robotRegion = getRegion(testRobotPos);
        int[] robotRegion = getRegion(new Translation3d(SwerveSubsystem.getInstance().getRobotPose().getX(), SwerveSubsystem.getInstance().getRobotPose().getY(), 0.08));
        int[][] top3Regions = new int[3][2];
        int[] bestRegion = new int[]{-1,-1};
        double maxRatio = 0;
        int first = 0;
        int second = 0;
        int third = 0;
        double firstDist = 0;
        double secondDist = 0;
        double thirdDist = 0;
        int currentFuelNum;
        
        for (int x = 0; x < searchRadius*2+1; x++) {
            for (int y = 0; y < searchRadius*2+1; y++) {
                currentFuelNum = numFuelsInRegion(new int[] {robotRegion[0]-searchRadius+x, robotRegion[1]-searchRadius+y});
                if (currentFuelNum > first) {
                    third = second;
                    top3Regions[2] = top3Regions[1];
                    
                    second = first;
                    top3Regions[1] = top3Regions[0];
                    
                    first = currentFuelNum;
                    top3Regions[0] = new int[] {robotRegion[0]-searchRadius+x, robotRegion[1]-searchRadius+y};
                } else if (currentFuelNum > second) {
                    third = second;
                    top3Regions[2] = top3Regions[1];
                    
                    second = currentFuelNum;
                    top3Regions[1] = new int[] {robotRegion[0]-searchRadius+x, robotRegion[1]-searchRadius+y};
                } else if (currentFuelNum > third) {
                    third = currentFuelNum;
                    top3Regions[2] = new int[] {robotRegion[0]-searchRadius+x, robotRegion[1]-searchRadius+y};
                }
            }
        }//got top 3 regions

        if (first == 0) {
            DriverCommunications.hasAutoDriveTarget = false;
            SwerveSubsystem.getInstance().hasAutoDriveTarget = false;
            SwerveSubsystem.getInstance().bestPlaceToGo = null;
            return null;
        }

        //first neighbors
        first += numFuelsInRegion(new int[] {top3Regions[0][0], top3Regions[0][1] + 1});
        first += numFuelsInRegion(new int[] {top3Regions[0][0], top3Regions[0][1] - 1});
        first += numFuelsInRegion(new int[] {top3Regions[0][0] + 1, top3Regions[0][1]});
        first += numFuelsInRegion(new int[] {top3Regions[0][0] -1, top3Regions[0][1]});
        //second neighbors
        second += numFuelsInRegion(new int[] {top3Regions[1][0], top3Regions[1][1] + 1});
        second += numFuelsInRegion(new int[] {top3Regions[1][0], top3Regions[1][1] - 1});
        second += numFuelsInRegion(new int[] {top3Regions[1][0] + 1, top3Regions[1][1]});
        second += numFuelsInRegion(new int[] {top3Regions[1][0] -1, top3Regions[1][1]});
        //third neighbors
        third += numFuelsInRegion(new int[] {top3Regions[2][0], top3Regions[2][1] + 1});
        third += numFuelsInRegion(new int[] {top3Regions[2][0], top3Regions[2][1] - 1});
        third += numFuelsInRegion(new int[] {top3Regions[2][0] + 1, top3Regions[2][1]});
        third += numFuelsInRegion(new int[] {top3Regions[2][0] -1, top3Regions[2][1]});

        //y/x
        firstDist = Math.max(1, Math.hypot(robotRegion[0] - top3Regions[0][0], robotRegion[1] - top3Regions[0][1]));
        secondDist = Math.max(1, Math.hypot(robotRegion[0] - top3Regions[1][0], robotRegion[1] - top3Regions[1][1]));
        thirdDist = Math.max(1, Math.hypot(robotRegion[0] - top3Regions[2][0], robotRegion[1] - top3Regions[2][1]));

        

        maxRatio = Math.max((first * (1/firstDist)), Math.max((second * (1/secondDist)), (third * (1/thirdDist))));

        if ((first * (1/firstDist)) == maxRatio) {
                bestRegion = top3Regions[0];
        } else if ((second * (1/secondDist)) == maxRatio) {
            bestRegion = top3Regions[1];
        } else if ((third * (1/thirdDist)) == maxRatio) {
            bestRegion = top3Regions[2];
        } 

        return new Translation3d(bestRegion[0] + 0.5, bestRegion[1] + 0.5, 0.08);
    }//returns nearest center of best region)

    public void driveToPosition (Translation3d pos) {
        Pose2d robotPose = SwerveSubsystem.getInstance().getRobotPose();
        double globalAngle = Math.toDegrees(Math.atan2(pos.getY() - robotPose.getY(), pos.getX() - robotPose.getX()));
        double change = globalAngle - robotPose.getRotation().getDegrees();
        change = Math.toDegrees(Math.atan2(
            Math.sin(Math.toRadians(change)), 
            Math.cos(Math.toRadians(change))
        ));
        
        double deltaX = pos.getX() - robotPose.getX();
        double deltaY = pos.getY() - robotPose.getY();

        double kPt = 0.5; //TODO: tune
        double speedX = deltaX * kPt;
        double speedY = deltaY * kPt;

        double kPr = 1; //TODO: tune
        double speedR = change * kPr;

        SwerveSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds(speedX, speedY, speedR));
    }


    //how to get which one i should go to?
    //max fuel for least distance
    //consider future trips?
    /*
     
        know where each fuel is
        know dists
        num fuel in area + num in neighbors?
        ok
        choose top 3 areas (m) w/ most
        add neighbor vals
        go to #1
        go to closest fuel within those neighbors



     */

    public void logPoses () {


        // SmartDashboard.putP
        SmartDashboard.putNumber("timeTest", Timer.getMatchTime());

        //new arr [#targets]
        //for #targets, set i = target pose

        // arrayPublisher.set(poses);//REAL
        poses = new Translation3d[] {
            new Translation3d(1,1,0.08),
            new Translation3d(1.1,1.1,0.08),
            new Translation3d(1.2,1,0.08),
            new Translation3d(1.3,1.1,0.08),
            new Translation3d(2,1,0.08),
            new Translation3d(2,1.2,0.08),
            new Translation3d(3,1,0.08),
            new Translation3d(3,1.1,0.08),
            new Translation3d(3,1.3,0.08),
            new Translation3d(4,1,0.08),
            new Translation3d(4,1,0.08),
            new Translation3d(0.9,1,0.08),
        };
        fuelLog.set(poses);
        best = bestPlaceToGo();
        if (best != null)
        bestPlaceToGo.set(new Pose3d(best.getX(), best.getY(), 0.08, new Rotation3d()));
        fakeRobot3d.accept(testRobotPos);
        
    }

    public void logHeatmap () {
        //blank for now
    }
}
/*
 
foreach target
get pose of target, add to list of time + pose

foreach time (every sec, 50 cycle)
if curr - time > max delete,
set list to pose + value, if .contains pose, use max

log pose



get # in sqr m, then if its > min, give pts
pts:
# * (1- (time since seen/max))
if rn, 999



 */