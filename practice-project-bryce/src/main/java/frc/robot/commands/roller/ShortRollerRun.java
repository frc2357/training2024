package frc.robot.commands.roller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShortRollerRun extends Command {
    private double m_speed;

    private Timer m_timer;

    public ShortRollerRun(double speed){
        m_speed = speed;
        m_timer = new Timer();
        
        addRequirements(Robot.roller);
    }

    @Override
    public void initialize(){
        System.out.println("Initialize ShortRollerRun");
        m_timer.start();
        Robot.roller.set(m_speed);
    }

    @Override
    public void execute() {}

    @Override 
    public boolean isFinished(){
        return m_timer.get() >= 5;
    }

    @Override 
    public void end(boolean interrupted){
        System.out.println("End ShortRollerRun");
        Robot.roller.stop();
        m_timer.reset();
        m_timer.stop();
    }
}
