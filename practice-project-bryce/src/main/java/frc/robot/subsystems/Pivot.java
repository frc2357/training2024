package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIVOT;

public class Pivot extends SubsystemBase {
    private CANSparkMax m_pivotMotor;

    private SparkPIDController m_pidController;

    private RelativeEncoder m_encoder;

    private int smartMotionSlot = 0;

    public Pivot() {
        m_pivotMotor = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless);
        m_pidController = m_pivotMotor.getPIDController();
        m_encoder = m_pivotMotor.getEncoder();
        configure();
    }

    public void configure() {
        m_pivotMotor.setInverted(PIVOT.MOTOR_INVERTED);
        m_pivotMotor.enableVoltageCompensation(12);
        m_pivotMotor.setSmartCurrentLimit(PIVOT.MOTOR_STALL_LIMIT_AMPS, PIVOT.MOTOR_FREE_LIMIT_AMPS);
        m_pivotMotor.setIdleMode(PIVOT.IDLE_MODE);

        m_pidController.setP(PIVOT.PIVOT_P);
        m_pidController.setI(PIVOT.PIVOT_I);
        m_pidController.setD(PIVOT.PIVOT_D);
        // m_pidController.setIZone(Constants.PIVOT.PIVOT_P);
        m_pidController.setFF(PIVOT.PIVOT_FF);
        m_pidController.setOutputRange(
            -1, 
            1);
        
        
        // m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        // m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        // m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(PIVOT.POSITION_ALLOWED_ERROR, smartMotionSlot);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void setPosition(double rotations) {
        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    public void set(double speed) {
        m_pivotMotor.set(speed);
    }

    public void zero() {
        m_encoder.setPosition(0);
    }

    public void stop() {
        set(0);
    }

    public void periodic() {
        //System.out.println(m_encoder.getPosition());
        SmartDashboard.putNumber("Motor Position", m_encoder.getPosition());
    }
}
