package frc.robot.subsystems.Lights;

import frc.robot.Constants;
import com.ctre.phoenix6.hardware.CANdle;

public class LightsIOSystem {
     CANdle candle = new CANdle(25, Constants.krakenBus);
}
