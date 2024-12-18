// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.signals.NeutralModeValue;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.CHOREO;
import frc.robot.Constants.CONTROLLER;
import frc.robot.commands.auto.CubeTestPath;
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
  public static AutoChooser autoChooser;

  private Command m_setCoastOnDisable;
  private Command m_allianceGetter;
  private Command m_forceGyroZero;

  private Map<String, Command> m_autoCommandsToBind = Map.of(
    
  );

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

    DriverStation.silenceJoystickConnectionWarning(true); // TODO: remove this if its a match

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
    m_forceGyroZero.cancel();
    m_setCoastOnDisable.cancel();
    Robot.swerve.configNeutralMode(NeutralModeValue.Brake);
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
     m_forceGyroZero.cancel();

    // Robot.swerve.setGyroOffset();
    m_setCoastOnDisable.cancel(); // very important, no touchy
    Robot.swerve.configNeutralMode(NeutralModeValue.Brake); // see above
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
