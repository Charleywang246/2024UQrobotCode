// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.UpperState;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoUpper;
import frc.robot.commands.Autos_X2;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  
  private final Swerve m_Swerve = new Swerve();
  private final UpperSub m_upper = new UpperSub();
  private final VisionSub m_vision = new VisionSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve tele = new TeleopSwerve(m_Swerve, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, driverController);
  private final Autos_X2 auto_x2 = new Autos_X2();
  private final AutoAim autoaim = new AutoAim(m_Swerve, m_upper, m_vision);

  // private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    m_Swerve.setDefaultCommand(tele);
    m_upper.setDefaultCommand(teleopUpper);

    // NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    // NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    // NamedCommands.registerCommand("print hello", Commands.print("Hello"));
    // NamedCommands.registerCommand("IntakeNote", new AutoUpper(m_upper, UpperState.GROUND, 0, 0, 0, 0));


    configureBindings();

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Mode", autoChooser);
    
  }

  private void configureBindings() {
    // SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
    //   new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
    //   new PathConstraints(
    //     4.0, 3.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   2.0
    // ));

    // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    //   new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
    //   new PathConstraints(
    //     4.0, 4.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   0
    // ));

    // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
    //   Pose2d currentPose = m_Swerve.getPose();
      
    //   // The rotation component in these poses represents the direction of travel
    //   Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    //   Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
    //   List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);

    //   PathPlannerPath path = new PathPlannerPath(
    //     bezierPoints, 
    //     new PathConstraints(
    //       4.0, 3.0, 
    //       Units.degreesToRadians(360), Units.degreesToRadians(540)
    //     ),  
    //     new GoalEndState(0.0, currentPose.getRotation())
    //   );

    //   // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    //   path.preventFlipping = false;

    //   AutoBuilder.followPath(path).schedule();
    // }));

    SmartDashboard.putData("Auto Aim", autoaim);
  }

  public Command getAutonomousCommand() {
    return auto_x2.getAuto(m_Swerve, m_upper);
  }
}
