package frc.robot.commands.auto;

// TODO: for later Choreo beta. no clue why it got pulled, but it was, so this is sitting here until its back.
public class CubeTestPath /*implements AutoRoutineGenerator*/{
  // // creates the routine that we need to use to make the path with triggers
  // final AutoRoutine m_routine = CHOREO.AUTO_FACTORY.newRoutine("Cube Test Path");
  // // loads the trajectory into the routine, use this to add triggers to the path
  // final AutoTrajectory m_cube = m_routine.trajectory("Cube test path");

  // final Alert m_noStartingPoseAlert = new Alert("Error: Cube Test Path has no starting pose.", AlertType.kError);

  // public CubeTestPath(){
  //   m_routine.running().onTrue(
  //     new InstantCommand(() -> {
  //       final Optional<Pose2d> initialPose = m_cube.getInitialPose();
  //       if(initialPose.isPresent()){
  //         Robot.swerve.setPose2d(initialPose.get());
  //       }
  //       m_noStartingPoseAlert.set(true);
  //       m_routine.kill();
  //     })// sets the odometry when the routine starts running.
  //   );
  //   m_cube.atTime(1.0).onTrue(new InstantCommand(() -> {System.out.println("1 second has passed since auto start");}));
  // }

  //   @Override
  //   public AutoRoutine apply(AutoFactory factory) {
  //     return m_routine;
  //   } // this returns the routine that this file uses when its added to the autoChooser

}
