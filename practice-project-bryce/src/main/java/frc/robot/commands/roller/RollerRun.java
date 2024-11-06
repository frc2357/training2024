package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class RollerRun extends Command {
    private double m_speed;

    public RollerRun(double speed){
        m_speed = speed;
        addRequirements(Robot.roller);
    }

    @Override
    public void initialize(){
        System.out.println("Initialize RollerRun");
        Robot.roller.set(m_speed);
    }

    @Override
    public void execute() {}

    @Override 
    public boolean isFinished(){
        return false;
    }

    @Override 
    public void end(boolean interrupted){
        System.out.println("End RollerRun");
        Robot.roller.stop();
    }
}
