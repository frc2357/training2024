package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.IntakeRun;

public class ParallelRun extends ParallelCommandGroup {
    public ParallelRun(double speed){
        super(new IntakeRun(speed), new RollerRun(speed));
    }
}
