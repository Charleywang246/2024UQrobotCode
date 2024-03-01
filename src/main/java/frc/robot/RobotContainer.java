// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.robotConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  
  private final Swerve m_Swerve = new Swerve();
  private final UpperSub m_upper = new UpperSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve tele = new TeleopSwerve(m_Swerve, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, driverController);

  public RobotContainer() {
    m_Swerve.setDefaultCommand(tele);
    m_upper.setDefaultCommand(teleopUpper);
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
