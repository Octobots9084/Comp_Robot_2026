package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.Shooter;

public class SwerveSubsystem extends SubsystemBase {


    private static SwerveSubsystem instance;
    /**
   * The wanted state of the swerve, which the subsystem attempts to set the {@link #currentState} to
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Drive.SwerveStates#MANUAL MANUAL}
   * @param SwerveStates The swerve states contain no information - {@link frc.robot.subsystems.Drive.SwerveStates SwerveStates}
   */
    public SwerveStates wantedState = SwerveStates.IDLE;
    /**
   * The current state of the swerve, which determines how the robot drives
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Drive.SwerveStates#IDLE IDLE}
   * @param SwerveStates The swerve states contain no information - {@link frc.robot.subsystems.Drive.SwerveStates SwerveStates}
   */
    private SwerveStates currentState = SwerveStates.IDLE;
    public SwerveIO io;
    public CommandXboxController driverController;
    public double maxVelocity;
    public double maxAngularVelocity;
    public double rotLockAngle = 0;
    // The robot pose estimator for tracking swerve odometry and applying vision corrections.

    public static PIDController angularPidcontroller = new PIDController(3, 0.5, 0);

    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    public SwerveSubsystem(
            SwerveIO io, CommandXboxController driverController, double maxAngularVelocity, double maxVelocity) {
        this.io = io;
        this.driverController = driverController;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxVelocity = maxVelocity;

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);///i uncommented all this and maybe it broke it idk
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
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
            if (currentPose.getX() <= 4.6) {// x boundary for blue alliance zone
                return true;
            }
            return false;
        } else {
            if (currentPose.getX() >= 11.9) {// x boundary for red alliance zone
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
        Logger.recordOutput("Xrot", this.io.getRotation3d().getX());
        Logger.recordOutput("Yrot", this.io.getRotation3d().getY());
        Logger.recordOutput("Zrot", this.io.getRotation3d().getZ());
        Logger.recordOutput("Tilt",
        Math.acos(this.io.getRotation3d().toMatrix().get(2, 2)));
        SmartDashboard.putBoolean("onRamp", isTilted(0, 3));
        applyStates();
        Logger.recordOutput("isInAllianceZone",this.isInAllianceZone());
        // Logger.recordOutput("front left absolute", io.getAbsoluteEncoderPositions(0));
        // Logger.recordOutput("front right absolute", io.getAbsoluteEncoderPositions(1));
        // Logger.recordOutput("back left absolute", io.getAbsoluteEncoderPositions(2));
        // Logger.recordOutput("back right absolute", io.getAbsoluteEncoderPositions(3));
    }

    public boolean isTilted(double wanted, double tolerance) { /////////////////////
        tolerance = Units.degreesToRadians(tolerance);
        Matrix<N3,N3> gyroMatrix = this.io.getRotation3d().toMatrix();
        
        // gyro mounted upside down so subtact PI radians out
        double tilt = Math.acos(gyroMatrix.get(2, 2)) - 0.015 - Math.PI; 
        return !MathUtil.isNear(wanted, tilt, tolerance);
    }

    public void registerNamedCommands () {

      NamedCommands.registerCommand("StartShoot", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTO;}));
      NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTONONFIRE;}));
      
      NamedCommands.registerCommand("DepotShoot", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTODEPOTSHOOT;}));
        
        NamedCommands.registerCommand("StartIntake", 
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> Intake.getInstance().alreadyZeroed),//so it doedsnt override auto init wanted=zero
                new InstantCommand(() -> {
                    Intake.getInstance().wantedState = IntakeStates.INTAKING;
                    Intake.getInstance().autoIntaked = true;
                })//,
            ).withTimeout(5)
        );
    
      NamedCommands.registerCommand("StartElephant", new InstantCommand(() -> {
        Intake.getInstance().wantedState = IntakeStates.ELEPHANTIASISPART2;
      }));

      NamedCommands.registerCommand("StopElephant", new InstantCommand(() -> {
        Intake.getInstance().wantedState = IntakeStates.EXTENDED;
      }));

      NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {Intake.getInstance().wantedState = IntakeStates.EXTENDED;}));
    //   NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> {Intake.getInstance().autonomousIntake = true;}));
    //   NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {Intake.getInstance().autonomousIntake = false;}));
        //TODO: i want to be ablel to just set the intake state to intaking, but it difnt work the first time, not confident it has to be done like this tho
        //(it does a loop instead of 1 set state)
    }

    private SwerveStates handleStateTransition() {
        switch (wantedState) {
            //redid how we handle the states that we switch to without modifications
            case MANUAL, IDLE, ROTATION_LOCK, REVERSE: 
                if (wantedState == SwerveStates.ROTATION_LOCK && currentState != SwerveStates.ROTATION_LOCK){
                    rotLockAngle = (Math.PI/2)*Math.round(getRobotPose().getRotation().getRadians()/(Math.PI/2));
                }
                return wantedState;
            case SLOW:
                if (currentState != SwerveStates.IDLE)
                    return wantedState;
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

                if (Shooter.driverOverride) 
                    wantedState = SwerveStates.SLOW;
                break;
            case SLOW:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                        .withSpeeds(calculateSpeedsBasedOnJoystickInputs().div(1.5))
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));

                if (!Shooter.driverOverride) 
                    wantedState = SwerveStates.MANUAL;
                break;
            case IDLE:

                break;
            case ROTATION_LOCK:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                        .withSpeeds(calculateRotLockSpeedsBasedOnJoystickInputs())
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                break;
            case REVERSE:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(-0.3, 0, 0))
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

    public ChassisSpeeds calculateRotLockSpeedsBasedOnJoystickInputs() {
        // double yMagnitude = MathUtil.applyDeadband(driverLeft.getRawAxis(0),
        // Constants.leftYDeadband);
        // double xMagnitude = -MathUtil.applyDeadband(driverLeft.getRawAxis(1),
        // Constants.leftXDeadband);
        // double angularMagnitude = -MathUtil.applyDeadband(driverRight.getRawAxis(0),
        // Constants.rightXDeadband);
        double yMagnitude = MathUtil.applyDeadband(driverController.getLeftX(), Constants.leftYDeadband);
        double xMagnitude = MathUtil.applyDeadband(driverController.getLeftY(), Constants.leftXDeadband);
        if (getRobotPose().getRotation().getRadians()-rotLockAngle<0.3){
            if (-MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband)>0.9){
                rotLockAngle = (rotLockAngle + Math.PI/2) % (Math.PI*2);
            }
            else if (-MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband)<-0.9){
                rotLockAngle = (rotLockAngle - Math.PI/2) % (Math.PI*2);
                
            }
        }
        double angularMagnitude = -MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband);
        double xVelocity = xMagnitude * maxVelocity;
        double yVelocity = yMagnitude * maxVelocity;

        double RotVelocity = angularPidcontroller.calculate(getRobotPose().getRotation().getRadians(),rotLockAngle);

        if (Constants.isBlueAlliance) {   
            return new ChassisSpeeds(-xVelocity, -yVelocity, RotVelocity);
        }
        return new ChassisSpeeds(xVelocity, yVelocity, RotVelocity);
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
