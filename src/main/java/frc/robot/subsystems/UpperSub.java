package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogTriggerOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.PID;
import frc.robot.Constants.LedContants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.UpperState;

public class UpperSub extends SubsystemBase{
        
    private final TalonFX leftElbow = new TalonFX(UpperConstants.leftElbowMotorID, robotConstants.canbusName);
    private final TalonFX rightElbow = new TalonFX(UpperConstants.rightElbowMotorID, robotConstants.canbusName);
    
    private final CANSparkMax leftShooter = new CANSparkMax(UpperConstants.leftShooterMotorID, MotorType.kBrushless);
    private final CANSparkMax rightShooter = new CANSparkMax(UpperConstants.rightShooterMotorID, MotorType.kBrushless);

    private final CANSparkMax intake = new CANSparkMax(UpperConstants.intakeMotorID, MotorType.kBrushless);

    private final CANcoder elbowCancoder = new CANcoder(UpperConstants.elbowCancoderID, robotConstants.canbusName);

    private AddressableLED led = new AddressableLED(LedContants.ledPwmPort);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LedContants.ledLenfth);
    private Timer timer = new Timer();

    // private final I2C.Port i2cPort = I2C.Port.kOnboard;
    // private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    // AnalogInput colorSensor = new AnalogInput(9);

    private UpperState state;

    private double elbowAngle;
    private double intakeSpeed;
    private double shooterSpeed;

    private final PID elbowPID = new PID(
        UpperConstants.elbowKP, 
        UpperConstants.elbowKI,
        UpperConstants.elbowKD,
        UpperConstants.elbowiWindup,
        UpperConstants.elbowiLimit
    );

    private final PID shooterPID = new PID(
        UpperConstants.shooterKP, 
        UpperConstants.shooterKI,
        UpperConstants.shooterKD,
        UpperConstants.shooteriWindup,
        UpperConstants.shooteriLimit
    );

    public UpperSub() {
        leftElbow.setInverted(false);
        rightElbow.setInverted(true);

        leftShooter.setInverted(false);
        rightShooter.setInverted(false);

        configElbow();

        led.setLength(LedContants.ledLenfth);
        setLED(new Color("#000000"));
        led.start();

        state = UpperState.DEFAULT;

    }

    // config
    public void configElbow() {
        elbowCancoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        );
    }

    // elbow
    public double getElbowRotation() {
        return elbowCancoder.getPosition().getValue();
    }

    public void setElbow(double speed) {
        leftElbow.set(speed);
        rightElbow.set(speed);
    }

    public double calculateElbowAngle() {
        return 0;
    }

    // intake
    public double getIntakeVel() {
        return intake.getEncoder().getVelocity();
    }

    public void setIntake(double speed) {
        intake.set(speed);
    }

    // shooter
    public void configShooter() {
        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();

        leftShooter.setIdleMode(IdleMode.kBrake);
        rightShooter.setIdleMode(IdleMode.kBrake);
    }

    public double getLeftShooterRPM() {
        return leftShooter.getEncoder().getVelocity();
    }

    public double getRightShooterRPM() {
        return rightShooter.getEncoder().getVelocity();
    }

    public double getShooterRPM() {
        return (getLeftShooterRPM() + getRightShooterRPM()) / 2;
    }

    public void setShooter(double speed) {
        leftShooter.set(speed);
        rightShooter.set(speed);
    }

    public void setShooter(double upSpeed, double downSpeed) {
        leftShooter.set(downSpeed);
        rightShooter.set(upSpeed);
    }

    public void setLED(Color color) {
        for(int i=0;i<buffer.getLength();i++) buffer.setRGB(i, (int)color.red, (int)color.green, (int)color.blue);
        led.setData(buffer);
    }

    public void blink(Color color) {
        timer.restart();
        if(timer.get() < 0.2) setLED(color);
        else setLED(new Color("#000000"));

        if(timer.get() > 0.4) timer.restart();
    }

    // color sensor
    // public double getHue() {
    //     frc.lib.math.Color color = new frc.lib.math.Color(
    //         colorSensor.getRed(),
    //         colorSensor.getGreen(),
    //         colorSensor.getBlue()
    //     );
    //     return color.getHue();
    // }

    // public boolean hasNote() {
    //     return Math.abs(getHue() - 30) < 10 ? true : false;

    // public boolean hasNote() {
    //     double val = colorSensor.getAverageVoltage();
    //     return true;
    // }

    // state machine

    public UpperState getState() {
        return state;
    }

    public void setState(UpperState state) {
        if(!UpperConstants.teleMode) {
            if(this.state == state && (state != UpperState.SHOOT)) {
                this.state = UpperState.DEFAULT;
            } else {
                this.state = state;
            } 
        }
    }

    @Override
    public void periodic() {

        switch (state) {
            case DEFAULT:
                elbowAngle = UpperConstants.ELBOW_DEFAULT_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                setLED(new Color("#E8D5F5"));
                break;
            case GROUND:
                elbowAngle = UpperConstants.ELBOW_GROUND_POS;
                intakeSpeed = UpperConstants.INTAKE_GROUND_SPEED;
                shooterSpeed = UpperConstants.SHOOTER_GROUND_SPEED;
                setLED(new Color("#0C29EB"));
                // if(hasNote()) setLED(new Color("#0C29EB"));
                // else blink(new Color("#0C29EB"));
                break;
            case AMP:
                elbowAngle = UpperConstants.ELBOW_AMP_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                setLED(new Color("#FFFF00"));
                break;
            case SPEAKER:
                elbowAngle = UpperConstants.ELBOW_SPEAKER_POS;
                intakeSpeed = 0;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                if(getShooterRPM() > 5000) setLED(new Color("#00FF00"));
                else setLED(new Color("#FF0000"));
                break;
            case SHOOT:
                intakeSpeed = UpperConstants.INTAKE_SHOOT_SPEED;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                blink(new Color("#00FF00"));
                break;
            case TELE:
                setLED(new Color("#4CE1C8"));
                break;
            case ENDGAME:
                elbowAngle = UpperConstants.ELBOW_GROUND_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                setLED(new Color("#9A00F3"));
        }

        if(!UpperConstants.teleMode) {
            setElbow(-elbowPID.calculate(elbowAngle - getElbowRotation()));
            setShooter(shooterSpeed);
            setIntake(intakeSpeed);
        }

        SmartDashboard.putString("robotState", state.toString());
        SmartDashboard.putNumber("elbowDEG", getElbowRotation());
        SmartDashboard.putNumber("intakeVel", getIntakeVel());
        SmartDashboard.putNumber("LeftShooterRPM", getLeftShooterRPM());
        SmartDashboard.putNumber("RightShooterRPM", getRightShooterRPM());
        // SmartDashboard.putNumber("Hue", getHue());
        // SmartDashboard.putBoolean("hasNote", hasNote());
        // SmartDashboard.putNumber("colorSensorInput", colorSensor.getAverageVoltage());

        SmartDashboard.putBoolean("tele", UpperConstants.teleMode);
    }
}