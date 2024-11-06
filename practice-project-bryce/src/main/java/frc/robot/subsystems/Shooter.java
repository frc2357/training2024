package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SHOOTER;
import frc.robot.Constants.CAN_ID;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_topShooterMotor;
    private SparkPIDController m_topPidController;
    private RelativeEncoder m_topEncoder;

    private CANSparkMax m_bottomShooterMotor;

    public Shooter() {
        m_topShooterMotor = new CANSparkMax(CAN_ID.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        m_topPidController = m_topShooterMotor.getPIDController();
        m_topEncoder = m_topShooterMotor.getEncoder();

        // m_bottomShooterMotor = new CANSparkMax(26, MotorType.kBrushless);
        configure();
    }

    public void configure() {
        m_topShooterMotor.setInverted(SHOOTER.TOP_MOTOR_INVERTED);
        m_bottomShooterMotor.setInverted(SHOOTER.BOTTOM_MOTOR_INVERTED);

        m_topShooterMotor.setOpenLoopRampRate(SHOOTER.RAMP_RATE);
        m_topShooterMotor.enableVoltageCompensation(12);
        m_topShooterMotor.setIdleMode(SHOOTER.IDLE_MODE);
        m_topShooterMotor.setSmartCurrentLimit(
                SHOOTER.TOP_MOTOR_STALL_LIMIT_AMPS,
                SHOOTER.TOP_MOTOR_FREE_LIMIT_AMPS);

        m_topPidController.setP(SHOOTER.TOP_MOTOR_P);
        m_topPidController.setI(SHOOTER.TOP_MOTOR_I);
        m_topPidController.setD(SHOOTER.TOP_MOTOR_D);

        // m_topPidController.setIZone(SHOOTER.TOP_MOTOR_I);
        m_topPidController.setFF(SHOOTER.TOP_MOTOR_FF);
        m_topPidController.setOutputRange(-1, 1);
        // m_bottomShooterMotor.enableVoltageCompensation(12);
        // m_bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        // m_bottomShooterMotor.setSmartCurrentLimit(60,60);
    }

    public double getVelocity() {
        return m_topEncoder.getVelocity();
    }

    public void setVelocity(double rpm) {
        m_topPidController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    public void set(double percentOutput) {
        m_topShooterMotor.set(percentOutput);
        // m_bottomShooterMotor.set(percentOutput);
    }

    public void stop() {
        set(0);
    }
}
