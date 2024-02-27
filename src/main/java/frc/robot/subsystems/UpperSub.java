package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.robotState;

public class UpperSub extends SubsystemBase{
    
    private final TalonFX leftElbow = new TalonFX(UpperConstants.leftElbowMotorID, robotConstants.canbusName);
    private final TalonFX rightElbow = new TalonFX(UpperConstants.rightElbowMotorID, robotConstants.canbusName);
    
    private final CANSparkMax leftShooter = new CANSparkMax(UpperConstants.leftShooterMotorID, MotorType.kBrushless);
    private final CANSparkMax rightShooter = new CANSparkMax(UpperConstants.rightShooterMotorID, MotorType.kBrushless);

    private final TalonSRX intake = new TalonSRX(UpperConstants.intakeMotorID);

    private final CANcoder elbowCancoder = new CANcoder(UpperConstants.elbowCancoderID, robotConstants.canbusName);
    private final CANcoder shooterCancoder = new CANcoder(UpperConstants.shooterCancoderID, robotConstants.canbusName);

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public UpperSub() {
        leftElbow.setInverted(false);
        rightElbow.setInverted(true);

        leftShooter.setInverted(false);
        rightShooter.setInverted(true);
    }

    // elbow
    public void resetElbow() {}

    public double getElbowRotation() {
        return elbowCancoder.getPosition().getValue();
    }

    public void setElbow(double speed) {
        leftElbow.set(speed);
        rightElbow.set(speed);
    }

    // intake
    public double getIntakeVel() {
        return intake.getSelectedSensorVelocity();
    }

    public void setIntake(double speed) {
        intake.set(TalonSRXControlMode.PercentOutput, speed);
    }

    // shooter
    public void resetShooter() {
        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();
    }

    public double getShooterRPM() {
        return shooterCancoder.getVelocity().getValue();
    }

    public void setShooter(double speed) {
        leftShooter.set(speed);
        rightShooter.set(speed);
    }

    public void setShooter(double upSpeed, double downSpeed) {
        leftShooter.set(downSpeed);
        rightShooter.set(upSpeed);
    }

    // color sensor
    public double getHue() {
        frc.lib.math.Color color = new frc.lib.math.Color(
            colorSensor.getRed(),
            colorSensor.getGreen(),
            colorSensor.getBlue()
        );
        return color.getHue();
    }

    public boolean hasNote() {
        return Math.abs(getHue() - 30) < 10 ? true : false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elbowDEG", getElbowRotation());
        // SmartDashboard.putNumber("intakeVel", getIntakeVel());
        // SmartDashboard.putNumber("shooterRPM", getShooterRPM());
        // SmartDashboard.putNumber("Hue", getHue());
        // SmartDashboard.putBoolean("hasNote", hasNote());
    }
}