package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase {
    private SparkMax m_motorLeft;
    private SparkMax m_motorRight;

    // private PIDController m_PidController

    public Elevator() {
        m_motorLeft = new SparkMax(Constants.CAN_ID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        m_motorRight = new SparkMax(Constants.CAN_ID.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);

        m_motorLeft.configure(Constants.ELEVATOR.LEFT_MOTOR_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_motorRight.configure(Constants.ELEVATOR.RIGHT_MOTOR_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public void setSpeed(double speed) {
        m_motorLeft.set(speed);
        m_motorRight.set(speed);

    }

    public void stop() {
        m_motorLeft.stopMotor();
        m_motorRight.stopMotor();
    }

}
