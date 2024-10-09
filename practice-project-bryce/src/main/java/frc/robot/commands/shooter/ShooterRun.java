package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterRun extends Command {
    private double m_speed;

    public ShooterRun(double speed){
        m_speed = speed;
        addRequirements(Robot.shooter);
    }

    @Override
    public void initialize(){
        System.out.println("Initialize ShooterRun");
        Robot.shooter.set(m_speed);
    }

    @Override
    public void execute() {}

    @Override 
    public boolean isFinished(){
        return false;
    }

    @Override 
    public void end(boolean interrupted){
        System.out.println("End ShooterRun");
        Robot.shooter.stop();
    }
}
