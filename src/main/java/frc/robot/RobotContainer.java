// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.robotConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.DriveForward;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
  
  private final Swerve m_Swerve = new Swerve();
  private final UpperSub m_upper = new UpperSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve tele = new TeleopSwerve(m_Swerve, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, driverController);

  private final Shoot X2 = new Shoot(m_upper, -0.204589);
  private final Shoot Y2 = new Shoot(m_upper, -0.147089);
  private final DriveForward X2Y2 = new DriveForward(m_Swerve, 1.5, -0.8, 0);
  private final DriveForward Y2Z3 = new DriveForward(m_Swerve, 1.5, -4.90, 0.9);
  private final DriveForward Z3Y2 = new DriveForward(m_Swerve, 1, 4.90, -1.44);
  private final AutoIntake intake = new AutoIntake(m_upper);

  public RobotContainer() {
    m_Swerve.setDefaultCommand(tele);
    m_upper.setDefaultCommand(teleopUpper);
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      X2,
      new ParallelRaceGroup(
        intake,
        X2Y2
      ),
      Y2,
      Y2Z3
    );
  }
}
