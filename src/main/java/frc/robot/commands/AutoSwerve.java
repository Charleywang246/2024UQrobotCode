package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

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

        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //     AutoConstants.kMaxSpeedMetersPerSecond,
        //     AutoConstants.kMaxAccelerationMetersPerSecondSquared
        // ).setKinematics(frc.robot.Constants.Swerve.swerveKinematics);

        String trajectoryJSON = "paths/YourPath.wpilib.json";
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SwerveControllerCommand(
            // TrajectoryGenerator.generateTrajectory(
            //     AutoConstants.target[id][0],
            //     new ArrayList<>(){{
            //         for(int i=1;i<AutoConstants.target[id].length-1;i++) add(AutoConstants.target[id][i].getTranslation());
            //     }},
            //     AutoConstants.target[id][AutoConstants.target[id].length - 1],
            //     trajectoryConfig
            // ),
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
