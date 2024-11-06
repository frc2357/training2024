package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class VelocityElevatorRun extends Command {
    public double m_rotations;

    public VelocityElevatorRun(double rotations){
        m_rotations = rotations;
        addRequirements(Robot.elevator);
    }

    public void initialize(){
        System.out.println("Initialize VelocityElevatorRun");
        // Robot.elevator.zero();
        // Robot.elevator.setVelocity(m_rotations);
    }

    @Override
    public void execute() {}

    @Override 
    public boolean isFinished(){
        return false;
    }

    @Override 
    public void end(boolean interrupted){
        System.out.println("End VelocityElevatorRun");
        Robot.elevator.stop();
    }
}
