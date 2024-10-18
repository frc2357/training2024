package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterRun extends Command {
    private double m_rpm;

    public ShooterRun(double rpm){
        m_rpm = rpm;
        addRequirements(Robot.shooter);
    }

    @Override
    public void initialize(){
        System.out.println("Initialize ShooterRun");
        Robot.shooter.setVelocity(m_rpm);
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
