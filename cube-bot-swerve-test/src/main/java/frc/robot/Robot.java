// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.features2d.MSER;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CONTROLLER;
import frc.robot.commands.drive.ForceGyroZero;
import frc.robot.commands.drive.SetCoastOnDisable;
import frc.robot.commands.state.GetAlliance;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.state.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static RobotContainer m_robotContainer;

  public static CommandSwerveDrivetrain swerve;
  public static RobotState state;
  public static DriverControls driverControls;

  private Command m_setCoastOnDisable;
  private Command m_allianceGetter;
  private Command m_forceGyroZero;


  @Override
  public void robotInit(){
    swerve = TunerConstants.createDrivetrain();
    state = new RobotState();
    driverControls = new DriverControls(new XboxController(CONTROLLER.DRIVE_CONTROLLER_PORT), CONTROLLER.DRIVE_CONTROLLER_DEADBAND);

    m_robotContainer = new RobotContainer();

    m_setCoastOnDisable = new SetCoastOnDisable();
    m_setCoastOnDisable.schedule();
    m_allianceGetter = new GetAlliance();
    m_allianceGetter.schedule();
    m_forceGyroZero = new ForceGyroZero();
    m_forceGyroZero.schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
