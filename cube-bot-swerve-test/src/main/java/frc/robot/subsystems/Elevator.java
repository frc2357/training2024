package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.Utility;

public class Elevator extends SubsystemBase {
    private SparkMax m_motorLeft;
    private SparkMax m_motorRight;
    SparkClosedLoopController m_PIDControllerLeft;
    SparkClosedLoopController m_PIDControllerRight;
    RelativeEncoder m_encoderLeft;
    RelativeEncoder m_encoderRight;
    double m_targetRotations;

    // private PIDController m_PidController

    public Elevator() {
        m_motorLeft = new SparkMax(Constants.CAN_ID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        m_motorRight = new SparkMax(Constants.CAN_ID.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);

        m_motorLeft.configure(Constants.ELEVATOR.MOTOR_CONFIG_RIGHT, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_motorRight.configure(Constants.ELEVATOR.MOTOR_CONFIG_LEFT, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_PIDControllerLeft = m_motorLeft.getClosedLoopController();
        m_PIDControllerRight = m_motorLeft.getClosedLoopController();

        m_encoderLeft = m_motorLeft.getEncoder();
        m_encoderRight = m_motorRight.getEncoder();
    }

    public void setSpeed(double speed) {
        m_motorLeft.set(speed);
        m_motorRight.set(speed);
        setTargetRotations(Double.NaN);

    }

    public void stop() {
        m_motorLeft.stopMotor();
        m_motorRight.stopMotor();
        setTargetRotations(Double.NaN);
    }

    public double getVelocity() {
        return m_encoderRight.getVelocity();
    }

    public void setZero() {
        m_encoderRight.setPosition(0);
        m_encoderLeft.setPosition(0);
    }

    public double getRotations() {
        return m_encoderRight.getPosition();
    }

    public void setTargetRotations(double targetRotations) {
        m_targetRotations = targetRotations;
        m_PIDControllerLeft.setReference(m_targetRotations, ControlType.kMAXMotionPositionControl);
        m_PIDControllerLeft.setReference(m_targetRotations, ControlType.kMAXMotionPositionControl);
    }

    public boolean isAtTargetRotations() {
        return Utility.isWithinTolerance(getRotations(), m_targetRotations,
                Constants.ELEVATOR.MAX_MOTION_ALLOWED_ERROR);
    };

}
