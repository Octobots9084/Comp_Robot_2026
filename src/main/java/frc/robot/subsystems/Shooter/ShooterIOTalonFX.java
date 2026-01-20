package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX{
    //fly wheel two x60
    //turret x60
    //hood x44
    //feeder x44(multi) x60(single)

    public TalonFX flyWheelLeftMotor;
    public TalonFX flyWheelRightMotor;
    public TalonFX turretMotor;
    public TalonFX hoodMotor;
    public TalonFX multiFeederMotor;
    public TalonFX singleFeederMotor;
    public ShooterConfigurator shooterConfigs;
    private MotionMagicVelocityVoltage flyWheelRequest;

    public ShooterIOTalonFX(){
        //TODO - set the canID's
        shooterConfigs = new ShooterConfigurator();
        flyWheelLeftMotor = new TalonFX(ShooterConstants.flyWheelLeftID,"krakenbus");
        flyWheelRightMotor = new TalonFX(0,"krakenbus");
        turretMotor = new TalonFX(0,"krakenbus");
        hoodMotor = new TalonFX(0,"krakenbus");
        multiFeederMotor = new TalonFX(0,"krakenbus");
        singleFeederMotor = new TalonFX(0,"krakenbus");

        flyWheelRightMotor.getConfigurator().apply(shooterConfigs.flyWheelRightConfig);
        turretMotor.getConfigurator().apply(shooterConfigs.turretConfig);
        hoodMotor.getConfigurator().apply(shooterConfigs.hoodConfig);
        multiFeederMotor.getConfigurator().apply(shooterConfigs.multiFeederConfig);
        singleFeederMotor.getConfigurator().apply(shooterConfigs.singleFeederConfig);
        flyWheelRequest = new MotionMagicVelocityVoltage(0);
    }
    public void setFlyWheelVelocity(double RPS){
        flyWheelRequest.Velocity = RPS;
        flyWheelRightMotor.setControl(flyWheelRequest);
        flyWheelRightMotor.setControl(new Follower(ShooterConstants.flyWheelRightID,MotorAlignmentValue.Opposed));
    }
}
