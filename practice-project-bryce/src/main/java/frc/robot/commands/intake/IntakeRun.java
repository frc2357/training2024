package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeRun extends Command {
    private double m_speed;

    public IntakeRun(double speed){
        m_speed = speed;
        addRequirements(Robot.intake);
    }

    @Override
    public void initialize(){
        System.out.println("Initialize");
        Robot.intake.set(m_speed);
    }

    @Override
    public void execute() {}

    @Override 
    public boolean isFinished(){
        return false;
    }

    @Override 
    public void end(boolean interrupted){
        Robot.intake.stop();
    }
}
