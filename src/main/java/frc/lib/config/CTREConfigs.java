package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants;

public final class CTREConfigs {
  public static CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {

  }

  public static CANcoderConfiguration CTREConfiguration(double angleOffset) {
    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    swerveCanCoderConfig.MagnetSensor.withSensorDirection(Constants.Swerve.canCoderInvert);
    swerveCanCoderConfig.MagnetSensor.withMagnetOffset(angleOffset);
    return swerveCanCoderConfig;
  }
}
