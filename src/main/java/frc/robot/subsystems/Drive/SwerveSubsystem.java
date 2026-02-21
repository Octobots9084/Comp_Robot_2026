package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.auto.DriveBack;
import frc.robot.commands.auto.DriveForwardUntilLevel;
import frc.robot.commands.auto.DriveOverBump;
import frc.robot.commands.auto.NoPoseBump.DriveOverBumpFromAlliance;
import frc.robot.commands.auto.NoPoseBump.DriveOverBumpToAlliance;
import frc.robot.subsystems.Vision.Vision;

public class SwerveSubsystem extends SubsystemBase{
    public enum SystemState {
        MANUAL,
        IDLE,
        ROTATION_LOCK,
        CLIMBALLIGN,
        REVERSE,
        ALIGNCLIMB
    }
    private static SwerveSubsystem instance;
    public SystemState wantedState = SystemState.MANUAL;
    public SystemState systemState = SystemState.IDLE;
    public SwerveIO io;
    public CommandXboxController driverController;
    public double maxVelocity;
    public double maxAngularVelocity;

    public int climbAllignStage = -1;

    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    public SwerveSubsystem(
        SwerveIO io, CommandXboxController driverController, double maxAngularVelocity, double maxVelocity
    ){
        this.io = io;
        this.driverController = driverController;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxVelocity = maxVelocity;

        // var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        // var visionStdDevs = VecBuilder.fill(1, 1, 1);
        // poseEstimator =
        //     new SwerveDrivePoseEstimator(
        //         new SwerveDriveKinematics(
        //             Constants.swerveModuleOneOffset,
        //             Constants.swerveModuleTwoOffset,
        //             Constants.swerveModuleThreeOffset,
        //             Constants.swerveModuleFourOffset),
        //         io.getGyroYaw(),
        //         io.getModulePositions(),
        //         new Pose2d(),
        //         stateStdDevs,
        //         visionStdDevs
        //     );
        instance = this;
        registerNamedCommands();
    }

    public static SwerveSubsystem getInstance() {
        if(instance == null)
            throw new IllegalStateException("Swerve instance not set");
        else
            return instance;
    }

    public static SwerveSubsystem setInstance(SwerveIO io, CommandXboxController driverController, double maxAngularVelocity, double maxVelocity) {
        SwerveSubsystem instance = new SwerveSubsystem(io,driverController,maxAngularVelocity,maxVelocity);
        instance.registerNamedCommands();
        return instance;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Swerve", inputs);
        systemState = handleStateTransition();
        // Logger.recordOutput("Xrot", this.io.getRotation3d().getX());
        // Logger.recordOutput("Yrot", this.io.getRotation3d().getY());
        // Logger.recordOutput("Zrot", this.io.getRotation3d().getZ());
        // Logger.recordOutput("Tilt", Math.acos(this.io.getRotation3d().toMatrix().get(2, 2)));
        SmartDashboard.putBoolean("onRamp", onRamp(0, 3));
        applyStates();
        Logger.recordOutput("front left absolute", io.getAbsoluteEncoderPositions(0));
        Logger.recordOutput("front right absolute", io.getAbsoluteEncoderPositions(1));
        Logger.recordOutput("back left absolute", io.getAbsoluteEncoderPositions(2));
        Logger.recordOutput("back right absolute", io.getAbsoluteEncoderPositions(3));
    }
    
    //TODO: move somewhere important
    public boolean onRamp (double wanted, double tolerance) { /////////////////////
      boolean inTolerance = false;
      tolerance = Units.degreesToRadians(tolerance);
      double tilt = Math.acos(this.io.getRotation3d().toMatrix().get(2, 2)) - 0.015;
      SmartDashboard.putNumber("Tilt", Units.radiansToDegrees(tilt));
      if (tilt <= (wanted + tolerance) && tilt >= (wanted - tolerance)) {
        inTolerance = true;
      }
      return !inTolerance;
    }

    public void registerNamedCommands () {
      NamedCommands.registerCommand("DriveOverBump",
                  new DriveOverBump());
                  /////////////////////
      NamedCommands.registerCommand("DriveOverBumpFromAlliance",
                  new DriveOverBumpFromAlliance());
      NamedCommands.registerCommand("DriveOverBumpToAlliance",
                  new DriveOverBumpToAlliance());
                  //////////////////////////
      
      NamedCommands.registerCommand("DriveBack",
                  new DriveBack().withTimeout(3));
        SmartDashboard.putBoolean("FinishedDriveForwardUntilLevel", false);

        SmartDashboard.putNumber("tilt", Math.acos(this.io.getRotation3d().toMatrix().get(2, 2)) - 0.015);

    }

    private SystemState handleStateTransition() {
        switch (wantedState){
            case MANUAL:
                return SystemState.MANUAL;
            case IDLE:
                return SystemState.IDLE;
            case ROTATION_LOCK:
                return SystemState.ROTATION_LOCK;
            case REVERSE:
                return SystemState.REVERSE;
            case ALIGNCLIMB:
                return SystemState.ALIGNCLIMB;
            case CLIMBALLIGN:
                if (systemState != SystemState.CLIMBALLIGN)
                climbAllignStage = -1;
                return SystemState.CLIMBALLIGN;
            default:
                return this.systemState;

        }
    }

    public void applyStates(){
        switch (systemState){
            case MANUAL:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                    .withSpeeds(calculateSpeedsBasedOnJoystickInputs())
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                break;
            case IDLE:
                
                break;
            case ROTATION_LOCK:
                
                break;
            case CLIMBALLIGN:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(Vision.getInstance().io.allignClimb(io.getPose2d(), climbAllignStage))
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                break;
            case REVERSE:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(-0.3, 0, 0))
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                break;
            case ALIGNCLIMB:
                
                break;
            default:
                break;

        }
    }

    public ChassisSpeeds calculateSpeedsBasedOnJoystickInputs(){
        // double yMagnitude = MathUtil.applyDeadband(driverLeft.getRawAxis(0), Constants.leftYDeadband);
        // double xMagnitude = -MathUtil.applyDeadband(driverLeft.getRawAxis(1), Constants.leftXDeadband);
        // double angularMagnitude = -MathUtil.applyDeadband(driverRight.getRawAxis(0), Constants.rightXDeadband);
        double yMagnitude = MathUtil.applyDeadband(driverController.getLeftX(), Constants.leftYDeadband);
        double xMagnitude = -MathUtil.applyDeadband(driverController.getLeftY(), Constants.leftXDeadband);
        double angularMagnitude = -MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);
        double xVelocity = (FieldConstants.isBlueAlliance() ? -xMagnitude * maxVelocity : xMagnitude * maxVelocity)
                * Constants.maxTelopVelocity;
        double yVelocity = (FieldConstants.isBlueAlliance() ? yMagnitude * maxVelocity : -yMagnitude * maxVelocity)
                * Constants.maxTelopVelocity;

        double angularVelocity = angularMagnitude * maxAngularVelocity * Constants.maxTelopAngularVelocity;

        if (Constants.allianceColor == Alliance.Blue) {
            return new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        }
        return new ChassisSpeeds(-xVelocity, -yVelocity, angularVelocity);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        this.io.driveFieldRelative(fieldRelativeSpeeds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        SmartDashboard.putNumberArray("Vision_STDEVS",stdDevs.getData());
        // poseEstimator.resetPose(visionMeasurement);

        io.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }
}
