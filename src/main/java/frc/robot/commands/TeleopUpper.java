package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.robotState;
import frc.robot.subsystems.UpperSub;

public class TeleopUpper extends Command{
    
    private final UpperSub sub;

    private final XboxController controller;

    private robotState state;

    private final PID elbowPID = new PID(
        UpperConstants.elbowKP, 
        UpperConstants.elbowKI,
        UpperConstants.elbowKD,
        UpperConstants.elbowiWindup,
        UpperConstants.elbowiLimit
    );

    private final PID shooterPID = new PID(
        UpperConstants.shooterKP, 
        UpperConstants.shooterKI,
        UpperConstants.shooterKD,
        UpperConstants.shooteriWindup,
        UpperConstants.shooteriLimit
    );

    public TeleopUpper(UpperSub sub, XboxController controller) {
        this.sub = sub;
        this.controller = controller;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        state = robotState.INIT;
    }

    @Override
    public void execute() {
        sub.setElbow(elbowPID.calculate(Constants.getElbowTarget(state) - sub.getElbowDeg()));
        sub.setShooter(shooterPID.calculate(Constants.getShooterTarget(state) - sub.getShooterVelocity()));
        sub.setIntake(Constants.getIntakeTarget(state));
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
