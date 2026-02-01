package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.auto.DriveBack;
import frc.robot.commands.auto.DriveForwardUntilLevel;
import frc.robot.commands.auto.DriveOverBump;

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
    public CommandXboxController driverController;
    public double maxVelocity;
    public double maxAngularVelocity;

    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    public SwerveSubsystem(
        SwerveIO io, CommandXboxController driverController, double maxAngularVelocity, double maxVelocity
    ){
        this.io = io;
        this.driverController = driverController;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxVelocity = maxVelocity;

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
      boolean inTolerance = true;
      tolerance = Units.degreesToRadians(tolerance);
      double tilt = Math.acos(this.io.getRotation3d().toMatrix().get(2, 2)) - 0.015;
      SmartDashboard.putNumber("Tilt", Units.radiansToDegrees(tilt));
      if (tilt <= (wanted + tolerance) && tilt >= (wanted - tolerance)) {
        inTolerance = false;
      }


      return inTolerance;
    }




    public void registerNamedCommands () {
      NamedCommands.registerCommand("DriveOverBump",
                  new DriveOverBump().andThen(new DriveForwardUntilLevel()));
      NamedCommands.registerCommand("DriveBack",
                  new DriveBack().withTimeout(3));
        SmartDashboard.putBoolean("FinishedDriveForwardUntilLevel", false);
                


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
        // double yMagnitude = MathUtil.applyDeadband(driverLeft.getRawAxis(0), Constants.leftYDeadband);
        // double xMagnitude = -MathUtil.applyDeadband(driverLeft.getRawAxis(1), Constants.leftXDeadband);
        // double angularMagnitude = -MathUtil.applyDeadband(driverRight.getRawAxis(0), Constants.rightXDeadband);
        double yMagnitude = MathUtil.applyDeadband(driverController.getLeftY(), Constants.leftYDeadband);
        double xMagnitude = -MathUtil.applyDeadband(driverController.getLeftX(), Constants.leftXDeadband);
        double angularMagnitude = -MathUtil.applyDeadband(driverController.getRawAxis(2), Constants.rightXDeadband);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);
        double xVelocity = (FieldConstants.isBlueAlliance() ? -xMagnitude * maxVelocity : xMagnitude * maxVelocity)
                * Constants.maxTelopVelocity;
        double yVelocity = (FieldConstants.isBlueAlliance() ? yMagnitude * maxVelocity : -yMagnitude * maxVelocity)
                * Constants.maxTelopVelocity;

        double angularVelocity = angularMagnitude * maxAngularVelocity * Constants.maxTelopAngularVelocity;

        return new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        this.io.driveFieldRelative(fieldRelativeSpeeds);
    }
}
