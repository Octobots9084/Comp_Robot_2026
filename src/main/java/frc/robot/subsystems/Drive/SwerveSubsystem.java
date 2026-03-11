package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.NamedCommands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.auto.DriveBack;
import frc.robot.commands.auto.DriveOverBump;
import frc.robot.commands.auto.NoPoseBump.DriveOverBumpFromAlliance;
import frc.robot.commands.auto.NoPoseBump.DriveOverBumpToAlliance;
import frc.robot.subsystems.Vision.VisionIOSystem;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class SwerveSubsystem extends SubsystemBase {


    private static SwerveSubsystem instance;
    public SwerveStates wantedState = SwerveStates.MANUAL;
    private SwerveStates currentState = SwerveStates.IDLE;
    public SwerveIO io;
    public CommandXboxController driverController;
    public double maxVelocity;
    public double maxAngularVelocity;
    // The robot pose estimator for tracking swerve odometry and applying vision corrections.
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    public SwerveSubsystem(
            SwerveIO io, CommandXboxController driverController, double maxAngularVelocity, double maxVelocity) {
        this.io = io;
        this.driverController = driverController;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxVelocity = maxVelocity;

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);///i uncommented all this and maybe it broke it idk
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
        instance = this;
        registerNamedCommands();
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null)
            throw new IllegalStateException("Swerve instance not set");
        else
            return instance;
    }

    public static SwerveSubsystem setInstance(SwerveIO io, CommandXboxController driverController,
            double maxAngularVelocity, double maxVelocity) {
        SwerveSubsystem instance = new SwerveSubsystem(io, driverController, maxAngularVelocity, maxVelocity);
        instance.registerNamedCommands();
        return instance;
    }

    public Pose2d getRobotPose() {
        return io.getPose2d();
    }

    public boolean isInAllianceZone() {
        Pose2d currentPose = this.getRobotPose();
        if (Constants.isBlueAlliance) {
            if (currentPose.getX() <= 3.67) {// x boundary for blue alliance zone
                return true;
            }
            return false;
        } else {
            if (currentPose.getX() >= 12.95) {// x boundary for red alliance zone
                return true;
            }
            return false;
        }
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Swerve", inputs);
        currentState = handleStateTransition();
        // Logger.recordOutput("Xrot", this.io.getRotation3d().getX());
        // Logger.recordOutput("Yrot", this.io.getRotation3d().getY());
        // Logger.recordOutput("Zrot", this.io.getRotation3d().getZ());
        // Logger.recordOutput("Tilt",
        // Math.acos(this.io.getRotation3d().toMatrix().get(2, 2)));
        // SmartDashboard.putBoolean("onRamp", onRamp(0, 3));
        applyStates();
        // Logger.recordOutput("front left absolute", io.getAbsoluteEncoderPositions(0));
        // Logger.recordOutput("front right absolute", io.getAbsoluteEncoderPositions(1));
        // Logger.recordOutput("back left absolute", io.getAbsoluteEncoderPositions(2));
        // Logger.recordOutput("back right absolute", io.getAbsoluteEncoderPositions(3));
    }

    // TODO: move somewhere important
    public boolean onRamp(double wanted, double tolerance) { /////////////////////
        boolean inTolerance = false;
        tolerance = Units.degreesToRadians(tolerance);
        Rotation3d gyroRotation = this.io.getRotation3d();
        Matrix<N3,N3> gyroMatrix = gyroRotation.toMatrix();
        double tilt = Math.acos(gyroMatrix.get(2, 2)) - 0.015 - Math.PI; // gyro mounted upside down so subtact PI radians out
        // SmartDashboard.putNumber("Tilt", Units.radiansToDegrees(tilt));
        if (tilt <= (wanted + tolerance) && tilt >= (wanted - tolerance)) {
            inTolerance = true;
        }
        return !inTolerance;
    }

    public void registerNamedCommands () {

      NamedCommands.registerCommand("StartShoot", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTO;}));
      NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTONONFIRE;}));
        

      NamedCommands.registerCommand("StartIntake", new InstantCommand(
        () -> {
            if (Intake.getInstance().currentState == IntakeStates.SAFE) {
                Intake.getInstance().currentState = IntakeStates.EXTENDED;
            }
            Intake.getInstance().wantedState = IntakeStates.INTAKING;
        }));
      NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {Intake.getInstance().wantedState = IntakeStates.EXTENDED;}));
    //   NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> {Intake.getInstance().autonomousIntake = true;}));
    //   NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {Intake.getInstance().autonomousIntake = false;}));
        //TODO: i want to be ablel to just set the intake state to intaking, but it difnt work the first time, not confident it has to be done like this tho
        //(it does a loop instead of 1 set state)
    }

    private SwerveStates handleStateTransition() {
        switch (wantedState) {
            case MANUAL:
                return SwerveStates.MANUAL;
            case IDLE:
                return SwerveStates.IDLE;
            case ROTATION_LOCK:
                return SwerveStates.ROTATION_LOCK;
            case REVERSE:
                return SwerveStates.REVERSE;
            case ALIGNCLIMB:
                if (this.currentState != SwerveStates.ALIGNCLIMB)
                    VisionIOSystem.climbAlignStage = 0;
                return SwerveStates.ALIGNCLIMB;
            default:
                return this.currentState;

        }
    }

    public void applyStates() {
        switch (currentState) {
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
            case ALIGNCLIMB:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(VisionIOSystem.allignClimb(getRobotPose()))
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                break;
            default:
                break;

        }
    }

    public ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        // double yMagnitude = MathUtil.applyDeadband(driverLeft.getRawAxis(0),
        // Constants.leftYDeadband);
        // double xMagnitude = -MathUtil.applyDeadband(driverLeft.getRawAxis(1),
        // Constants.leftXDeadband);
        // double angularMagnitude = -MathUtil.applyDeadband(driverRight.getRawAxis(0),
        // Constants.rightXDeadband);
        double yMagnitude = MathUtil.applyDeadband(driverController.getLeftX(), Constants.leftYDeadband);
        double xMagnitude = MathUtil.applyDeadband(driverController.getLeftY(), Constants.leftXDeadband);
        double angularMagnitude = -MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);
        double xVelocity = xMagnitude * maxVelocity;
        double yVelocity = yMagnitude * maxVelocity;

        double angularVelocity = angularMagnitude * maxAngularVelocity;

        if (Constants.isBlueAlliance) {   
            return new ChassisSpeeds(-xVelocity, -yVelocity, angularVelocity);
        }
        return new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        this.io.driveFieldRelative(fieldRelativeSpeeds);
    }

    /**
     * See
     * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        // SmartDashboard.putNumberArray("Vision_STDEVS", stdDevs.getData());
        // poseEstimator.resetPose(visionMeasurement);

        io.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    public SwerveStates getCurrentState() {
        return this.currentState;
    }
}
