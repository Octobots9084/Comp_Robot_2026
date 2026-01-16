package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
    // public boolean connected = false;
    // public Rotation2d yawPosition = Rotation2d.kZero;
    // public double yawVelocityRadPerSec = 0.0;
    // public double[] odometryYawTimestamps = new double[] {};
    // public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    
    @Override
    public void toLog(LogTable table){

    }

    @Override
    public void fromLog(LogTable table) {

    }

}
