package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Constants.CHOREO.AUTO_FACTORY;

public class Autos {

  public Autos(){

  }
  
  public AutoRoutine cubeTestPath(){
    AutoRoutine routine = AUTO_FACTORY.newRoutine("CubeTestPath");
    
    AutoTrajectory cubeTestPathTraj = routine.trajectory("CubeTestPath");
    
    routine.active().onTrue(
      Commands.sequence(
        AUTO_FACTORY.resetOdometry("CubeTestPath"),
        new InstantCommand(() -> System.out.println("runnign the commands")),
        cubeTestPathTraj.cmd()
      )
    );

    return routine;
  }
}
