package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class IntakeRunAxis extends Command {
    private AxisInterface m_speed;

    public IntakeRunAxis(AxisInterface speed){
        m_speed = speed;
        addRequirements(Robot.intake);
    }

    @Override
    public void initialize(){
        System.out.println("Initialize IntakeRun");
        Robot.intake.set(m_speed.getValue());
    }

    @Override
    public void execute() {
        Robot.intake.set(m_speed.getValue());
    }

    @Override 
    public boolean isFinished(){
        return false;
    }

    @Override 
    public void end(boolean interrupted){
        System.out.println("End IntakeRun");
        Robot.intake.stop();
    }
}
