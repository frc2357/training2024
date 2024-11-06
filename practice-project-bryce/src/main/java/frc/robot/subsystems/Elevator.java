package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.ELEVATOR;

public class Elevator extends SubsystemBase {
    private CANSparkMax m_leftElevatorMotor;
    private SparkPIDController m_leftPidController;
    private RelativeEncoder m_leftEncoder;

    private CANSparkMax m_rightElevatorMotor;

    private int smartMotionSlot = 0;

    public Elevator(){
        m_leftElevatorMotor = new CANSparkMax(CAN_ID.LEFT_ELEVATOR_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        m_leftPidController = m_leftElevatorMotor.getPIDController();
        m_leftEncoder = m_leftElevatorMotor.getEncoder();

        m_rightElevatorMotor = new CANSparkMax(CAN_ID.RIGHT_ELEVATOR_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        
        configure();
    }

    public void configure(){
        m_leftElevatorMotor.setInverted(ELEVATOR.MOTOR_INVERTED);
        m_leftElevatorMotor.enableVoltageCompensation(12);
        m_leftElevatorMotor.setIdleMode(ELEVATOR.IDLE_MODE);
        m_leftElevatorMotor.setSmartCurrentLimit(
            ELEVATOR.MOTOR_STALL_LIMIT_AMPS, 
            ELEVATOR.MOTOR_FREE_LIMIT_AMPS);

        m_leftPidController.setP(ELEVATOR.PIVOT_P);
        m_leftPidController.setI(ELEVATOR.PIVOT_I);
        m_leftPidController.setD(ELEVATOR.PIVOT_D);
        m_leftPidController.setIZone(ELEVATOR.PIVOT_I_ZONE);
        m_leftPidController.setFF(ELEVATOR.PIVOT_FF);
        m_leftPidController.setOutputRange(-0.5, 0.5);
        
        m_leftPidController.setSmartMotionMaxVelocity(ELEVATOR.MAX_VELOCITY, smartMotionSlot);
        m_leftPidController.setSmartMotionMinOutputVelocity(ELEVATOR.MIN_OUTPUT_VELOCTIY, smartMotionSlot);
        m_leftPidController.setSmartMotionMaxAccel(ELEVATOR.MAX_ACCELERATION, smartMotionSlot);
        m_leftPidController.setSmartMotionAllowedClosedLoopError(ELEVATOR.ALLOWED_ERROR, smartMotionSlot);

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
