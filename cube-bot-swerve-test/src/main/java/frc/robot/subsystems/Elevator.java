package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase {
    private SparkMax m_motorLeft;
    private SparkMax m_motorRight;
    private SparkMaxConfig m_configLeft;
    private SparkMaxConfig m_configRight;

    // private PIDController m_PidController

    public Elevator() {
        m_motorLeft = new SparkMax(Constants.CAN_ID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        m_motorRight = new SparkMax(Constants.CAN_ID.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);

        m_configLeft = new SparkMaxConfig();
        m_configRight = new SparkMaxConfig();

        m_configLeft.idleMode(IdleMode.kBrake);
        m_configLeft.inverted(true);

        m_configRight.idleMode(IdleMode.kBrake);
        m_configRight.inverted(false);

        m_motorLeft.configure(m_configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorRight.configure(m_configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setSpeed(double speed) {
        m_motorLeft.set(speed);
        m_motorRight.set(-speed);

    }

}
