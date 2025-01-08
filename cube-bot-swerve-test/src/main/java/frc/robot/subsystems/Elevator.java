package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase {
    private SparkMax m_motor_left;
    private SparkMax m_motor_right;

    // private PIDController m_PidController

    public Elevator() {
        m_motor_left = new SparkMax(Constants.CAN_ID.Elevator.LEFT_MOTOR, MotorType.kBrushless);
        m_motor_right = new SparkMax(Constants.CAN_ID.Elevator.RIGHT_MOTOR, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        m_motor_left.set(speed);
        m_motor_right.set(-speed);
    }

}
