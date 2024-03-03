package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.UpperSub;

public class AutoUpper extends Command{

    private final UpperSub sub;
    private final UpperState state;
    private final double elbowAngle, intakeSpeed, shooterSpeed;
    private final double time;
    private final Timer timer = new Timer();

    public AutoUpper(UpperSub sub, UpperState state, double elbowAngle, double intakeSpeed, double shooterSpeed, double time) {
        this.sub = sub;
        this.state = state;
        this.elbowAngle = elbowAngle;
        this.intakeSpeed = intakeSpeed;
        this.shooterSpeed = shooterSpeed;
        this.time = time;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        DriverStation.reportWarning("AutoUpper Command Start: mode " + state.toString(), false);
        sub.setState(state);
        timer.restart();
    }

    @Override
    public void execute() {
        if(sub.getState() == UpperState.TELE) {
            sub.setElbowAngle(elbowAngle);
            sub.setIntake(intakeSpeed);
            sub.setShooter(shooterSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        sub.setState(UpperState.DEFAULT);
        DriverStation.reportWarning("AutoUpper Command End: mode " + state.toString(), false);
    }

    @Override
    public boolean isFinished() {
        if(state != UpperState.GROUND && timer.get() > time) return true;
        else return false;
    }
}
