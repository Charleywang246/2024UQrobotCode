package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;

public class Autos_X2 extends Command{

    public SequentialCommandGroup getAuto(Swerve swerve, UpperSub upper) {
        return new SequentialCommandGroup(
            new AutoUpper(upper, UpperState.SPEAKER, 0, 0, 0, 1.5),
            new AutoUpper(upper, UpperState.SHOOT, 0, 0, 0, 0.5),
            new ParallelCommandGroup(
                new AutoUpper(upper, UpperState.GROUND, 0, 0, 0, 1.3),
                new AutoSwerve(swerve, -0.8, 0, 1)
            ),
            new AutoUpper(upper, UpperState.TELE, -0.152089, 0, -1, 1),
            new AutoUpper(upper, UpperState.SHOOT, 0, 0, 0, 0.5)
            // new ParallelCommandGroup(
            //     new AutoUpper(upper, UpperState.GROUND, 0, 0, 0, 1.5),
            //     new AutoSwerve(swerve, -4.90, 1.1, 1.4)
            // ),
            // new ParallelCommandGroup(
            //     new AutoUpper(upper, UpperState.GROUND, 0, 0, 0, 1.5),
            //     new AutoSwerve(swerve, -0.816, 0.23, 1.2)
            // ),
            // new ParallelCommandGroup(
            //     new AutoUpper(upper, UpperState.TELE, -0.147089, 0, -1, 1.5),
            //     new AutoSwerve(swerve, 4.90, -1.1, 1.5)
            // ),
            // new AutoUpper(upper, UpperState.TELE, -0.147089, 1, -1, 0.5)
        );
    }
}
