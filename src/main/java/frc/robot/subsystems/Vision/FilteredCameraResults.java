package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonTrackedTargetProto;

import org.photonvision.EstimatedRobotPose;

public class FilteredCameraResults {
    public ArrayList<Optional<EstimatedRobotPose>> multiTagHubResults;
    public ArrayList<Optional<EstimatedRobotPose>> multiTagResults;
    public ArrayList<Optional<EstimatedRobotPose>> singleTagResults;

    public ArrayList<Optional<List<PhotonTrackedTarget>>> multiTagHubTargets;
    public ArrayList<Optional<List<PhotonTrackedTarget>>> multiTagTargets;
    public ArrayList<Optional<List<PhotonTrackedTarget>>> singleTagTargets;
    
    public FilteredCameraResults(
        ArrayList<Optional<EstimatedRobotPose>> MultiTagHubResults,
        ArrayList<Optional<EstimatedRobotPose>> MultiTagResults,
        ArrayList<Optional<EstimatedRobotPose>> singleTagResults,
        ArrayList<Optional<List<PhotonTrackedTarget>>> MultiTagHubTargets,
        ArrayList<Optional<List<PhotonTrackedTarget>>> MultiTagTargets,
        ArrayList<Optional<List<PhotonTrackedTarget>>> singleTagTargets
        ){
        this.multiTagHubResults = MultiTagHubResults;
        this.multiTagResults = MultiTagResults;
        this.singleTagResults = singleTagResults;
        this.multiTagHubTargets = MultiTagHubTargets;
        this.multiTagTargets = MultiTagTargets;
        this.singleTagTargets = singleTagTargets;
    }
}
