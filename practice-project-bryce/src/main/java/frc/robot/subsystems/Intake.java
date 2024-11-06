package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.CAN_ID;

public class Intake extends SubsystemBase {
    private CANSparkMax m_topIntakeMotor;
    private CANSparkMax m_bottomIntakeMotor;

    public Intake(){
        m_topIntakeMotor = new CANSparkMax(CAN_ID.TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
        m_bottomIntakeMotor = new CANSparkMax(CAN_ID.BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);
        configure();
    }

    public void configure(){
        m_topIntakeMotor.setInverted(INTAKE.TOP_MOTOR_INVERTED);
        m_bottomIntakeMotor.setInverted(INTAKE.BOTTOM_MOTOR_INVERTED);

        m_topIntakeMotor.enableVoltageCompensation(12);
        m_topIntakeMotor.setIdleMode(INTAKE.IDLE_MODE);
        m_topIntakeMotor.setSmartCurrentLimit(
            INTAKE.TOP_MOTOR_STALL_LIMIT_AMPS,
            INTAKE.TOP_MOTOR_FREE_LIMIT_AMPS);
    
        m_bottomIntakeMotor.enableVoltageCompensation(12);
        m_bottomIntakeMotor.setIdleMode(INTAKE.IDLE_MODE);
        m_bottomIntakeMotor.setSmartCurrentLimit(
            INTAKE.BOTTOM_MOTOR_STALL_LIMIT_AMPS, 
            INTAKE.BOTTOM_MOTOR_FREE_LIMIT_AMPS);
    }

    public void set(double percentOutput){
        m_topIntakeMotor.set(percentOutput);
        m_bottomIntakeMotor.set(percentOutput);
    }

    public void stop() {
        set(0);
    }
}
