package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;

public class AutoGo extends Command{
    private final Swerve swerve;
    double x,y,z;

    private final PID xyPID = new PID(0.03, 0, 0, 0, 0);
    private final PID zPID = new PID(0.001, 0, 0, 0, 0);

    public AutoGo(Swerve swerve, double x, double y, double z) {
        this.swerve = swerve;
        this.x = x;
        this.y = y;
        this.z = z;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(
            new Translation2d(xyPID.calculate(x - swerve.getPose().getX()), xyPID.calculate(y - swerve.getPose().getY())), 
            zPID.calculate(z - swerve.getPose().getRotation().getRotations()), true, false
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        if(
            Math.abs(x - swerve.getPose().getX()) < 0.05 &&
            Math.abs(y - swerve.getPose().getY()) < 0.05 &&
            Math.abs(z - swerve.getPose().getRotation().getRotations()) < 0.005
        )  return true;
        return false;
    }
}
