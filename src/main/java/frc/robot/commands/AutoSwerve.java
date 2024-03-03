package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AutoSwerve extends Command{
    
    private final Swerve swerve;

    private double xSpeed, ySpeed, time;

    private final Timer timer = new Timer();

    public AutoSwerve(Swerve swerve, double xSpeed, double ySpeed, double time) {
        this.swerve = swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.time = time;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        DriverStation.reportError("AutoSwerve Command Start:\nxSpeed: "+xSpeed+" ySpeed"+ySpeed+" Time:"+time, false);
        timer.restart();
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(xSpeed, ySpeed), 0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, false);
        timer.stop();
        DriverStation.reportError("AutoSwerve Command End:\nxSpeed: "+xSpeed+" ySpeed"+ySpeed+" Time:"+time, false);
    }

    @Override
    public boolean isFinished() {
        if(timer.get() > time) return true;
        else return false;
    }
}
