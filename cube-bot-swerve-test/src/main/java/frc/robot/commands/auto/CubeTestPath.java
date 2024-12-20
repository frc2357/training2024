package frc.robot.commands.auto;

import static frc.robot.Constants.CHOREO.AUTO_FACTORY;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CHOREO;

public class CubeTestPath {

  public CubeTestPath(){
  }

  public AutoRoutine getRoutine(){
    AutoRoutine routine = CHOREO.AUTO_FACTORY.newRoutine("CubeTestPath");
    AutoTrajectory cube = routine.trajectory("CubeTestPath");
    routine.active().onTrue(AUTO_FACTORY.resetOdometry("CubeTestPath"));
    return routine;
  }

  public Command getCommand(){
    AutoRoutine routine = CHOREO.AUTO_FACTORY.newRoutine("CubeTestPath");
    AutoTrajectory cube = routine.trajectory("CubeTestPath");
    routine.active().onTrue(AUTO_FACTORY.resetOdometry("CubeTestPath"));
    return routine.cmd();
  }

}
