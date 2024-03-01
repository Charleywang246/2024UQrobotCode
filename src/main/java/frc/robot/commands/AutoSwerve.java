package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class AutoSwerve extends Command{

    private final Swerve m_Swerve;

    public AutoSwerve(Swerve swerve) {
        this.m_Swerve = swerve;
        addRequirements(swerve);
    }

    public SwerveControllerCommand getCommand(int id) {

        Trajectory trajectory = new Trajectory();

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SwerveControllerCommand(
            trajectory,
            m_Swerve::getPose,
            frc.robot.Constants.Swerve.swerveKinematics,
            xController,
            yController,
            thetaController,
            m_Swerve::setModuleStates,
            m_Swerve
        );
    }
}
