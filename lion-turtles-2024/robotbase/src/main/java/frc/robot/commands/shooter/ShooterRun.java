// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterRun extends Command {
  private double m_velocity;

  /**
   * Creates a new Shooter command.
   *
   * @param rpm The velocity to set the shooter at.
   */
  public ShooterRun(double rpm) {
    m_velocity = rpm;

    addRequirements(Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooter.setVelocity(m_velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
