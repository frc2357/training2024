package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax m_topIntakeMotor;
    private CANSparkMax m_bottomIntakeMotor;

    public Intake(){
        m_topIntakeMotor = new CANSparkMax(23, MotorType.kBrushless);
        m_bottomIntakeMotor = new CANSparkMax(24, MotorType.kBrushless);
        configure();
    }

    public void configure(){
        m_topIntakeMotor.setInverted(false);
        m_bottomIntakeMotor.setInverted(false);

        m_topIntakeMotor.enableVoltageCompensation(12);
        m_topIntakeMotor.setIdleMode(IdleMode.kCoast);
        m_topIntakeMotor.setSmartCurrentLimit(60,60);
    
        m_bottomIntakeMotor.enableVoltageCompensation(12);
        m_bottomIntakeMotor.setIdleMode(IdleMode.kCoast);
        m_bottomIntakeMotor.setSmartCurrentLimit(60,60);
    }

    public void set(double percentOutput){
        m_topIntakeMotor.set(percentOutput);
        m_bottomIntakeMotor.set(percentOutput);
    }

    public void stop() {
        set(0);
    }
}
