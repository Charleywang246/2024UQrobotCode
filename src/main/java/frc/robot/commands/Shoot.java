package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants.UpperConstants;
import frc.robot.subsystems.UpperSub;

public class Shoot extends Command{
    UpperSub sub;
    Timer timer = new Timer();

    double targetRotation;

    private final PID elbowPID = new PID(
        UpperConstants.elbowKP,
        UpperConstants.elbowKI,
        UpperConstants.elbowKD,
        UpperConstants.elbowiWindup,
        UpperConstants.elbowiLimit
    );

    public Shoot(UpperSub sub, double targetRotation) {
        this.sub = sub;
        this.targetRotation = targetRotation;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        DriverStation.reportError("Shoot" + targetRotation + "start", false);
    }

    @Override
    public void execute() {
        sub.setElbow(elbowPID.calculate(sub.getElbowRotation() - targetRotation));
        sub.setShooter(-1);
        if(Math.abs(sub.getElbowRotation() - targetRotation) < 0.005) {
            sub.setIntake(1);
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        sub.setShooter(0);
        sub.setIntake(0);
        sub.setElbow(0);
        timer.stop();
        DriverStation.reportError("Shoot" + targetRotation + "end", false);
    }

    @Override
    public boolean isFinished() {
        if(timer.get() > 1) return true;
        else return false;
    }
}
