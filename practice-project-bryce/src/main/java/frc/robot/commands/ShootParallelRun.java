package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.shooter.ShooterRun;

public class ShootParallelRun extends ParallelCommandGroup {
    public ShootParallelRun(double speed){
        super(new IntakeRun(speed), new ShooterRun(speed / 4));
    }
}
