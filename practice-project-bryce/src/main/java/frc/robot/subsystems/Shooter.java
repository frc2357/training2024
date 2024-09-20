package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_topShooterMotor;
    private CANSparkMax m_bottomShooterMotor;

    public Shooter(){
        m_topShooterMotor = new CANSparkMax(25, MotorType.kBrushless);
        m_bottomShooterMotor = new CANSparkMax(26, MotorType.kBrushless);
    }

    public void configure(){
        m_topShooterMotor.setInverted(false);
        m_bottomShooterMotor.setInverted(false);

        m_topShooterMotor.enableVoltageCompensation(12);
        m_topShooterMotor.setIdleMode(IdleMode.kCoast);
        m_topShooterMotor.setSmartCurrentLimit(60,60);
    
        m_bottomShooterMotor.enableVoltageCompensation(12);
        m_bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        m_bottomShooterMotor.setSmartCurrentLimit(60,60);
    }

    public void set(double percentOutput){
        m_topShooterMotor.set(percentOutput);
        m_bottomShooterMotor.set(percentOutput);
    }

    public void stop() {
        set(0);
    }
}
