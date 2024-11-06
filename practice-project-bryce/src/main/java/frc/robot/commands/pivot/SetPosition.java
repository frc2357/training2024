package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetPosition extends Command {
    private double m_rotations;

    public SetPosition(double rotations){
        m_rotations = rotations;
        addRequirements(Robot.pivot);
    }

    public void initialize(){
        System.out.println("Initialize SetPosition");
        Robot.pivot.zero();
        Robot.pivot.setPosition(m_rotations);
    }

    @Override
    public void execute() {}

    @Override 
    public boolean isFinished(){
        return false; //Math.abs(m_rotations - Robot.pivot.getPosition()) < 10;
    }

    @Override 
    public void end(boolean interrupted){
        System.out.println("End SetPosition");
        Robot.pivot.stop();
    }
}
