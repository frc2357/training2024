package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private CANSparkMax m_leftElevatorMotor;
    private SparkPIDController m_leftPidController;
    private RelativeEncoder m_leftEncoder;

    private CANSparkMax m_rightElevatorMotor;

    private int smartMotionSlot = 0;

    public Elevator(){
        m_leftElevatorMotor = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless);
        m_leftPidController = m_leftElevatorMotor.getPIDController();
        m_leftEncoder = m_leftElevatorMotor.getEncoder();

        m_rightElevatorMotor = new CANSparkMax(31, CANSparkLowLevel.MotorType.kBrushless);
        
        configure();
    }

    public void configure(){
        m_leftElevatorMotor.setInverted(false);
        m_leftElevatorMotor.enableVoltageCompensation(12);
        m_leftElevatorMotor.setIdleMode(IdleMode.kBrake);
        m_leftElevatorMotor.setSmartCurrentLimit(60, 60);

        m_leftPidController.setP(0.001);
        m_leftPidController.setI(0.00000001);
        m_leftPidController.setD(0);
        m_leftPidController.setIZone(0);
        m_leftPidController.setFF(0);
        m_leftPidController.setOutputRange(-0.5, 0.5);
        
        m_leftPidController.setSmartMotionMaxVelocity(2500, smartMotionSlot);
        m_leftPidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        m_leftPidController.setSmartMotionMaxAccel(1500, smartMotionSlot);
        m_leftPidController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

        m_rightElevatorMotor.follow(m_leftElevatorMotor);
    }

    // @Deprecated(forRemoval=true)
    // @Deprecated(forRemoval=false)
    public double getPosition() {
        return m_leftEncoder.getPosition();
    }
    
    public double getVelocity() {
        return m_leftEncoder.getVelocity();
    }

    public void setPosition(double rotations){
        m_leftPidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    public void setVelocity(double rotations){
        m_leftPidController.setReference(rotations, CANSparkMax.ControlType.kVelocity);
    }

    public void set(double speed) {
        // System.out.println("Speed: " + speed);
        m_leftElevatorMotor.set(speed);
    }

    public void zero() {
        m_leftEncoder.setPosition(0);
    }

    public void stop() {
        set(0);
    }
}
