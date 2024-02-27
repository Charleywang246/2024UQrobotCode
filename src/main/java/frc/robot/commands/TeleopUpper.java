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

    // private boolean shooting = false;
    // private boolean intaking = false;

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
        //         intakeSpeed = 0;
        //         shooterSpeed = 0;
        //         break;
        //     case GROUND:
        //         elbowAngle = UpperConstants.ELBOW_GROUND_POS;
        //         intakeSpeed = sub.hasNote() ? 0 : UpperConstants.INTAKE_GROUND_SPEED;
        //         shooterSpeed = sub.hasNote() ? 0 : UpperConstants.SHOOTER_GROUND_SPEED;
        //         break;
        //     case AMP:
        //         elbowAngle = UpperConstants.ELBOW_AMP_POS;
        //         intakeSpeed = 0;
        //         shooterSpeed = 0;
        //         break;
        //     case SPEAKER:
        //         elbowAngle = UpperConstants.ELBOW_SPEAKER_POS;
        //         intakeSpeed = 0;
        //         shooterSpeed = 0;
        //         break;
        //     case SHOOT:
        //         intakeSpeed = UpperConstants.INTAKE_SHOOT_SPEED;
        //         shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
        //         break;
        // }

        // 😂😂

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
            double elbowTarget = ground ? UpperConstants.ELBOW_GROUND_POS : UpperConstants.ELBOW_DEFAULT_POS;
            double elbowOutput = elbowPID.calculate(elbowTarget - sub.getElbowRotation());
            sub.setElbow(elbowOutput);
            SmartDashboard.putNumber("output", elbowOutput);
        }

        SmartDashboard.putBoolean("tele", tele);
        SmartDashboard.putBoolean("ground", ground);
        SmartDashboard.putString("robotState", state.toString());

        // 😂😂

        // if(controller.getRightBumperPressed()) {
        //     state = robotState.GROUND;
        // }

        // if(state == robotState.GROUND && sub.hasNote()) {
        //     state = robotState.DEFAULT;
        // }

        // if(controller.getLeftBumperPressed()) {
        //     state = robotState.SHOOT;
        // }

        // if(controller.getXButtonPressed()) {
        //     state = robotState.AMP;
        // }

        // if(controller.getRightTriggerAxis() > 0.5) {
        //     state = robotState.SHOOT;
        // }

        // if(state == robotState.SHOOT && !sub.hasNote()) {
        //     state = robotState.DEFAULT;
        // }

        // sub.setElbow(elbowPID.calculate(elbowAngle - sub.getElbowRotation()));
        // sub.setIntake(intakeSpeed);
        // sub.setShooter(shooterPID.calculate(shooterSpeed - sub.getShooterRPM()));

        // 😂😂

        // if(!intaking && !shooting) {
        //     sub.setIntake(-controller.getLeftY());
        //     sub.setShooter(-controller.getRightY());
        // }

        // SmartDashboard.putNumber("setLeft", -controller.getLeftY());
        // SmartDashboard.putNumber("getLeftVel", sub.getIntakeVel());
        // SmartDashboard.putNumber("setRight", -controller.getRightY());
        // SmartDashboard.putNumber("getShooterRPM", sub.getShooterRPM());

        // SmartDashboard.putString("robotState", state.toString());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
