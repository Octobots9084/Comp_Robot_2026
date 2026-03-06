package frc.robot.subsystems.Shooter.Turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConfigurator;

public class TurretIOTalonFX implements TurretIO {
    public TalonFX hoodMotor;
    public TalonFX turretMotor;
    public double zeroTurret;
    private MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
    private MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
    public DigitalInput turretMagnetBreak = new DigitalInput(1);
    public double deadZoneTolerance = 0.1;
    public double wrapPoint = 0;
    public boolean aimed;
    public boolean aimedToShoot;

    public ShooterConfigurator shooterConfigs = new ShooterConfigurator();

    public TurretIOTalonFX() {
        shooterConfigs = new ShooterConfigurator();
        hoodMotor = new TalonFX(Constants.hoodID, Constants.krakenBus);
        turretMotor = new TalonFX(Constants.turretID, Constants.krakenBus);
        turretMotor.setPosition(0);
        hoodMotor.getConfigurator().apply(shooterConfigs.hoodConfig);
        hoodMotor.setPosition(85/360.0);
        turretMotor.getConfigurator().apply(shooterConfigs.turretConfig);
        
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretLimitSwitch = turretMagnetBreak.get();
        inputs.hoodMotorTemp = hoodMotor.getDeviceTemp().getValueAsDouble();
        inputs.turretMotorTemp = turretMotor.getDeviceTemp().getValueAsDouble();
        inputs.hoodPosition = this.getHoodPosition()*360;
        inputs.turretPosition = this.getTurretPosition()*360;
        inputs.hoodRequest = hoodRequest.Position*360;
        inputs.turretRequest = turretRequest.Position*360;
        inputs.turretVoltage = this.turretMotor.getMotorVoltage().getValueAsDouble();
        inputs.turretCurrent = this.turretMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setTurretPosition(double turretAngle) {
        // aimedToShoot = false;
        // if (turretAngle < Constants.minTurretAngle/(Math.PI*2)) {
        //     turretMotor.setControl(turretRequest.withPosition(Constants.minTurretAngle/(Math.PI*2)));
        // }else if(turretAngle > Constants.maxTurretAngle/(Math.PI*2)){
        //     turretMotor.setControl(turretRequest.withPosition(Constants.maxTurretAngle/(Math.PI*2)));
        // }else{   
        //     aimedToShoot = true;
            turretMotor.setControl(turretRequest.withPosition(turretAngle));
        // }



    }

    @Override
    public boolean getAimedToShoot(){
        return aimedToShoot;
    }

    @Override
    public void setHoodPosition(double hoodAngle) {
        // double hoodAngleAfterCompensation = 1.39131*hoodAngle -32.83666;
        hoodAngle = Math.max(hoodAngle, Constants.minimumHoodPosition);
        hoodAngle = Math.min(hoodAngle, Constants.maximumHoodPosition);
        hoodMotor.setControl(hoodRequest.withPosition(hoodAngle));
    }

    @Override
    public double getHoodPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
        
    }

    @Override
    public double getTurretPosition() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    @Override
    public boolean hoodInTolerance(double tolerance) {
        return MathUtil.isNear(hoodRequest.getPositionMeasure().in(Units.Revolution), this.getHoodPosition(),
                tolerance);
    }

    @Override
    public boolean turretInTolerance(double tolerance) {
        return MathUtil.isNear(turretRequest.getPositionMeasure().in(Units.Revolution), this.getTurretPosition(),
                tolerance);
    }

    /** loop this if it is being used */

    double startTime = -1000;
    double duration = 1000;

    public void startFiring() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean turretZeroed() {
        if (!turretMagnetBreak.get() || Shooter.getInstance().alreadyZeroed) {
            turretMotor.setVoltage(0);
            turretMotor.setPosition(192/360.0);
            this.setTurretPosition(0);
            return true;
        } else {
            turretMotor.setVoltage(1);// was 3v
            return false;
        }
    }
}
