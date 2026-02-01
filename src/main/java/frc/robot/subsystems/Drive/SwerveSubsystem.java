package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class SwerveSubsystem extends SubsystemBase{
    public enum SystemState {
        MANUAL,
        IDLE,
        ROTATION_LOCK,
        REVERSE,
        ALIGN
    }
    private static SwerveSubsystem instance;
    public SystemState wantedState = SystemState.MANUAL;
    public SystemState systemState = SystemState.IDLE;
    public SwerveIO io;
    public CommandJoystick driverLeft;
    public CommandJoystick driverRight;
    public double maxVelocity;
    public double maxAngularVelocity;
    // The robot pose estimator for tracking swerve odometry and applying vision corrections.
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    public SwerveSubsystem(
        SwerveIO io, CommandJoystick driverLeft, CommandJoystick driverRight, double maxAngularVelocity, double maxVelocity
    ){
        this.io = io;
        this.driverLeft = driverLeft;
        this.driverRight = driverRight;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxVelocity = maxVelocity;

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        poseEstimator =
            new SwerveDrivePoseEstimator(
                new SwerveDriveKinematics(
                    Constants.swerveModuleOneOffset,
                    Constants.swerveModuleTwoOffset,
                    Constants.swerveModuleThreeOffset,
                    Constants.swerveModuleFourOffset),
                io.getGyroYaw(),
                io.getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionStdDevs
            );
    }

    public static SwerveSubsystem getInstance() {
        if(instance == null)
            throw new IllegalStateException("Swerve instance not set");
        else
            return instance;
    }

    public static SwerveSubsystem setInstance(SwerveIO io, CommandJoystick driverLeft, CommandJoystick driverRight, double maxAngularVelocity, double maxVelocity) {
        return instance = new SwerveSubsystem(io,driverLeft,driverRight,maxAngularVelocity,maxVelocity);
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Swerve", inputs);
        systemState = handleStateTransition();
        applyStates();
        Logger.recordOutput("front left absolute", io.getAbsoluteEncoderPositiosn(0));
        Logger.recordOutput("front right absolute", io.getAbsoluteEncoderPositiosn(1));
        Logger.recordOutput("back left absolute", io.getAbsoluteEncoderPositiosn(2));
        Logger.recordOutput("back right absolute", io.getAbsoluteEncoderPositiosn(3));
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
            case ALIGN:
                return SystemState.ALIGN;
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
            case REVERSE:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(-0.3, 0, 0))
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                break;
            case ALIGN:
                
                break;
            default:
                break;

        }
    }
    public ChassisSpeeds calculateSpeedsBasedOnJoystickInputs(){
        double yMagnitude = MathUtil.applyDeadband(driverLeft.getRawAxis(1), Constants.leftYDeadband);
        double xMagnitude = MathUtil.applyDeadband(driverLeft.getRawAxis(0), Constants.leftXDeadband);
        double angularMagnitude = MathUtil.applyDeadband(driverRight.getRawAxis(0), Constants.rightXDeadband);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);
        double xVelocity = (FieldConstants.isBlueAlliance() ? -xMagnitude * maxVelocity : xMagnitude * maxVelocity)
                * Constants.maxTelopVelocity;
        double yVelocity = (FieldConstants.isBlueAlliance() ? yMagnitude * maxVelocity : -yMagnitude * maxVelocity)
                * Constants.maxTelopVelocity;

        double angularVelocity = angularMagnitude * maxAngularVelocity * Constants.maxTelopAngularVelocity;

        return new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }
}
