package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class DriveForward extends Command{

    private final Swerve swerve;

    private final Timer timer = new Timer();
    double targetTime = 0;
    double x,y;

    public DriveForward(Swerve swerve, double targetTime,double x,double y) {
        this.swerve = swerve;
        this.targetTime = targetTime;
        this.x = x;
        this.y = y;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.restart();
        System.out.println("Move" + targetTime + "start");
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(x, y), 0, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.drive(new Translation2d(0, 0), 0, false, false);
        System.out.println("Move" + targetTime + "end");
    }

    @Override
    public boolean isFinished() {
        if(timer.get() > targetTime) return true;
        else return false;
    }
}
