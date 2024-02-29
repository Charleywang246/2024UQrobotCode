// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.AutoSwerve;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
  
  private final Swerve m_Swerve = new Swerve();
  private final UpperSub m_upper = new UpperSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve tele = new TeleopSwerve(m_Swerve, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, driverController);
  // private final AutoSwerve autoSwerve = new AutoSwerve(m_Swerve);

  // private final SendableChooser<Command> autoChooser;
  

  public RobotContainer() {
    m_Swerve.setDefaultCommand(tele);
    m_upper.setDefaultCommand(teleopUpper);
    configureBindings();

    // autoChooser = AutoBuilder.buildAutoChooser();
  }

  private void configureBindings() {

  //   SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
  //     Pose2d currentPose = m_Swerve.getPose();
      
  //     // The rotation component in these poses represents the direction of travel
  //     Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
  //     Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

  //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
  //     PathPlannerPath path = new PathPlannerPath(
  //       bezierPoints, 
  //       new PathConstraints(
  //         4.4, 3, 
  //         Units.degreesToRadians(360), Units.degreesToRadians(540)
  //       ),  
  //       new GoalEndState(0.0, currentPose.getRotation())
  //     );

  //     // Prevent this path from being flipped on the red alliance, since the given positions are already correct
  //     path.preventFlipping = true;

  //     AutoBuilder.followPath(path).schedule();
  //   }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    // return autoChooser.getSelected();
  }
}
