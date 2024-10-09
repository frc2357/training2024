package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private CANSparkMax m_pivotMotor;

    private SparkPIDController m_pidController;

    private RelativeEncoder m_encoder;

    private double kP = 1.5;
    private double kI = 0; //0.00000001;
    private double kD = 0;
    private double kIz = 0;
    private double kFF = 0;
    private double kMaxOutput = 0.25;
    private double kMinOutput = -0.25;

    private double maxVel = 4000;
    private double minVel;
    private double maxAcc = 50000;
    private double allowedErr;

    private int smartMotionSlot = 0;

    public Pivot() {
        m_pivotMotor = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless);
        m_pidController = m_pivotMotor.getPIDController();
        m_encoder = m_pivotMotor.getEncoder();
        configure();
    }

    public void configure() {
        m_pivotMotor.setInverted(false);
        m_pivotMotor.enableVoltageCompensation(12);
        m_pivotMotor.setIdleMode(IdleMode.kBrake);

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        
        m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
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
