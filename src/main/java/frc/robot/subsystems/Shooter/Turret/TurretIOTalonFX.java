package frc.robot.subsystems.Shooter.Turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;

public class TurretIOTalonFX implements TurretIO{
    public TalonFX hoodMotor;
    public TalonFX turretMotor;       
    public double zeroTurret;
    private MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
    private MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
    // public DigitalInput turretLimitSwitch = new DigitalInput(0);
    public double deadZoneTolerance = 0.1;
    public double wrapPoint = 0;
    
    public ShooterConfigurator shooterConfigs = new ShooterConfigurator();

    public TurretIOTalonFX(){
        shooterConfigs = new ShooterConfigurator();
        hoodMotor = new TalonFX(Constants.hoodID, Constants.krakenBus);
        turretMotor = new TalonFX(Constants.turretID,Constants.krakenBus);

        hoodMotor.getConfigurator().apply(shooterConfigs.hoodConfig);
        turretMotor.getConfigurator().apply(shooterConfigs.turretConfig);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs){
        // inputs.hoodaMotorTemp = hoodMotor.getDeviceTemp().getValueAsDouble();
        // inputs.turretMotorTemp = turretMotor.getDeviceTemp().getValueAsDouble();
        // inputs.hoodPosition = this.getHoodPosition();
        // inputs.turretPosition = this.getTurretPosition();
        // inputs.hoodRequest = hoodRequest.Position;
        // inputs.turretRequest = turretRequest.Position;
    }

    @Override
    public void setTurretPosition(double turretAngle){
        turretAngle = Math.max(turretAngle, 0);
        turretAngle = Math.min(turretAngle, Constants.maximumTurretPosition);
        turretRequest.Position = turretAngle;
        turretMotor.setControl(turretRequest);
    }

    @Override
    public void setHoodPosition(double hoodAngle){
        hoodAngle = Math.max(hoodAngle, 0);
        hoodAngle = Math.min(hoodAngle, Constants.maximumHoodPosition);
        hoodRequest.Position = hoodAngle;
        hoodMotor.setControl(hoodRequest);
    }

    @Override
    public double getHoodPosition(){
        return hoodMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getTurretPosition(){
        return turretMotor.getPosition().getValueAsDouble();
    }
    
    @Override
    public boolean hoodInTolerance(double tolerance){
        return MathUtil.isNear(hoodRequest.getPositionMeasure().in(Units.Revolution), this.getHoodPosition(), tolerance);
    }

    @Override
    public boolean turretInTolerance(double tolerance){
        return MathUtil.isNear(turretRequest.getPositionMeasure().in(Units.Revolution), this.getTurretPosition(), tolerance);
    }

    /** loop this if it is being used */
    
    double startTime = -1000;
    double duration = 1000;

    public void startFiring() {
        startTime = System.currentTimeMillis();
    }

    // public boolean turretZeroed(){
    //     if(turretLimitSwitch.get()){
    //         turretMotor.setVoltage(0);
    //         zeroTurret = turretMotor.getPosition().getValueAsDouble();            
    //         return true;
    //     }else{
    //         turretMotor.setVoltage(1);//TODO set this to real value Santi!
    //         return false;
    //     }
    // }
}
