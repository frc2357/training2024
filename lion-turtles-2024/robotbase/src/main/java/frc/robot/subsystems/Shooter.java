// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SHOOTER;

/** A subsystem used to control the shooter part of the ping pong launcher. */
public final class Shooter extends SubsystemBase {
  private CANSparkMax m_leftShooterMotor;
  private SparkPIDController m_leftPidController;
  private RelativeEncoder m_leftEncoder;

  // private CANSparkMax m_rightShooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    m_leftShooterMotor = new CANSparkMax(CAN_ID.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    m_leftPidController = m_leftShooterMotor.getPIDController();
    m_leftEncoder = m_leftShooterMotor.getEncoder();

    // m_rightShooterMotor = new CANSparkMax(CAN_ID.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

    configure();
  }

  private void configure() {
    m_leftShooterMotor.setInverted(SHOOTER.LEFT_MOTOR_INVERTED);

    m_leftShooterMotor.setOpenLoopRampRate(SHOOTER.RAMP_RATE);
    m_leftShooterMotor.enableVoltageCompensation(12);
    m_leftShooterMotor.setIdleMode(SHOOTER.IDLE_MODE);
    m_leftShooterMotor.setSmartCurrentLimit(
        SHOOTER.LEFT_MOTOR_STALL_LIMIT_AMPS,
        SHOOTER.LEFT_MOTOR_FREE_LIMIT_AMPS);
    m_leftPidController.setP(SHOOTER.LEFT_MOTOR_P);
    m_leftPidController.setI(SHOOTER.LEFT_MOTOR_I);
    m_leftPidController.setD(SHOOTER.LEFT_MOTOR_D);
    m_leftPidController.setIZone(SHOOTER.LEFT_MOTOR_I_ZONE);
    m_leftPidController.setFF(SHOOTER.LEFT_MOTOR_FF);
    m_leftPidController.setOutputRange(-1, 1);

    // m_rightShooterMotor.follow(m_leftShooterMotor, SHOOTER.RIGHT_MOTOR_INVERTED_FROM_LEFT);
  }

  /**
   * Gets the velocity of the shooter in RPM.
   *
   * @return the number representing the RPM of the shooter.
   */
  public double getVelocity() {
    return m_leftEncoder.getVelocity();
  }

  /**
   * Sets the velocity of the shooter using RPM.
   *
   * @param rpm The units in RPM to set.
   */
  // @Deprecated(forRemoval=true)
  public void setVelocity(double rpm) {
    m_leftPidController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  /** Stops the shooter from running. */
  public void stop() {
    m_leftShooterMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
