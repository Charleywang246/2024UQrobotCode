// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class robotConstants {
    public static final String canbusName = "GTX7130";
    public static final int DriverControllerID = 0;
  }

  public enum UpperState {
    DEFAULT,
    GROUND,
    AMP,
    SPEAKER,
    SHOOT,
    TELE
  }

  public static final class Swerve {
    public static final double axisDeadBand = 0.05; // make sure ur robot won't vibrate cuz the joystick gives a input like 0.002 or sth
    public static final int pigeon1 = 13; // advanced gyro
    public static final int pigeon2 = 14;
    public static final int pigeon3 = 15;
    public static final int pigeon4 = 16;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.583; // meters, length between two side's wheels, need to adjust
    public static final double wheelBase = 0.583; // meters, length between same side's wheels, need to adjust
    public static final double wheelDiameter = Units.inchesToMeters(4.0); // need to adjust
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;  // open loop means no feedback(PID), closed loop vise versa, not used actually

    public static final double driveGearRatio = (6.12244897959 / 1.0); // 6.12:1 (6.12244897959), for MK4i(L3)
    public static final double angleGearRatio = (150.0 / 7.0 / 1.0); // 150/7 : 1, for MK4i(all)

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    ); // locating Swerve's positions, notice the sequences(first is 0, second is 1, etc.)

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0; // setting the nominal voltage(won't really follow anyway)

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 30; //20
    public static final int driveContinuousCurrentLimit = 40; //80, limiting the amps so Neo won't brake

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKD = 0.0; // maybe need to adjust

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0025;
    public static final double driveKFF = 0.0; // maybe need to adjust

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27; // feedforward, maybe need to adjust

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio; // like constants in physics

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.4; // meters per second
    public static final double maxAngularVelocity = 13.5; // meters per second

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kCoast; // whether u want to let neo stop slowly individually(coast) or fiercely wholely(brake)

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false; // yeah invert the motor

    /* Angle Encoder Invert */
    public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; // invert cancoder(in CTREconfig)

    /* Field Oriented */
    // public static boolean fieldOriented = false;
    
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.291016);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.166992);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Rear Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.609375);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Rear Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.025879);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

  }

  public static final class VisionConstants {
    public static final String CAMname = "";
    public static final Transform3d RCtoCAM = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Translation3d zeroTranslation3d = new Translation3d(0, 0, 0);
    public static final Transform3d zeroTransform3d = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    public static final Transform3d Id47ToSpeakerC = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d Id38ToSpeakerC = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
  }

  public static final class UpperConstants {
    public static final int leftElbowMotorID = 17;
    public static final int rightElbowMotorID = 18;
    public static final int leftShooterMotorID = 21;
    public static final int rightShooterMotorID = 20;
    public static final int elbowCancoderID = 19;
    public static final int intakeMotorID = 22;
    // public static final int shooterCancoderID = 1;

    public static final double shooter_arm_Angle = 135;

    public static final double elbowKP = 10;
    public static final double elbowKI = 0.0;
    public static final double elbowKD = 5;
    public static final double elbowiWindup = 0.0;
    public static final double elbowiLimit = 0.0;

    public static final double shooterKP = 0.0;
    public static final double shooterKI = 0.0;
    public static final double shooterKD = 0.0;
    public static final double shooteriWindup = 0.0;
    public static final double shooteriLimit = 0.0;

    public static final double ELBOW_DEFAULT_POS = -0.002197;
    public static final double ELBOW_GROUND_POS = -0.236328;
    public static final double ELBOW_AMP_POS = 0.012939;
    public static final double ELBOW_SPEAKER_POS = -0.204589;

    public static final double INTAKE_GROUND_SPEED = -0.4;
    public static final double INTAKE_SHOOT_SPEED = -1;

    public static final double SHOOTER_GROUND_SPEED = 0.02;
    public static final double SHOOTER_SHOOT_SPEED = -1;
    public static final double SHOOTER_LEGAL_SPEED = 5000;
  }

  public static final class LedContants {
    public static final int ledLenfth = 34;
    public static final int ledPwmPort = 9;
  }

}