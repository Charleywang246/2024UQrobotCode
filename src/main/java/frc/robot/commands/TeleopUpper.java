package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double elbowAngle;
    private double intakeSpeed;
    private double shooterSpeed;

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

    private boolean tele = false;
    private boolean ground = false;

    public TeleopUpper(UpperSub sub, XboxController controller) {
        this.sub = sub;
        this.controller = controller;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        state = robotState.DEFAULT;
    }

    @Override
    public void execute() {

        // switch (state) {
        //     case DEFAULT:
        //         elbowAngle = UpperConstants.ELBOW_DEFAULT_POS;
        //         intakeSpeed = UpperConstants.INTAKE_DEFAULT_SPEED;
        //         shooterSpeed = UpperConstants.SHOOTER_DEFAULT_SPEED;
        //         break;
        //     case GROUND:
        //         elbowAngle = UpperConstants.ELBOW_GROUND_POS;
        //         intakeSpeed = UpperConstants.INTAKE_GROUND_SPEED;
        //         shooterSpeed = UpperConstants.SHOOTER_GROUND_SPEED;
        //         break;
        //     case AMP:
        //         elbowAngle = UpperConstants.ELBOW_AMP_POS;
        //         intakeSpeed = UpperConstants.INTAKE_AMP_SPEED;
        //         shooterSpeed = UpperConstants.SHOOTER_AMP_SPEED;
        //         break;
        //     case SPEAKER:
        //         elbowAngle = UpperConstants.ELBOW_SPEAKER_POS;
        //         intakeSpeed = UpperConstants.INTAKE_SPEAKER_SPEED;
        //         shooterSpeed = UpperConstants.SHOOTER_SPEAKER_SPEED;
        //         break;
        //     case SHOOT:
        //         break;
        // }

        if(controller.getLeftBumperPressed()) {
            tele = !tele;
        }

        if(controller.getRightBumperPressed()) {
            ground = !ground;
        }

        if(tele) {
            double ep = MathUtil.applyDeadband(controller.getLeftTriggerAxis(), 0.05) > 0.05 ?
             MathUtil.applyDeadband(controller.getLeftTriggerAxis(), 0.05) : 
             MathUtil.applyDeadband(-controller.getRightTriggerAxis(), 0.05);
            sub.setElbow(ep);
        } else {
            double target = ground ? UpperConstants.ELBOW_GROUND_POS : UpperConstants.ELBOW_DEFAULT_POS;
            double output = elbowPID.calculate(target - sub.getElbowRotation());
            sub.setElbow(output);
            SmartDashboard.putNumber("output", output);
        }
        // sub.setShooter(-controller.getRightY());
        // sub.setIntake(controller.getLeftTriggerAxis());

        SmartDashboard.putBoolean("tele", tele);
        SmartDashboard.putBoolean("ground", ground);
        SmartDashboard.putString("robotState", state.toString());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
