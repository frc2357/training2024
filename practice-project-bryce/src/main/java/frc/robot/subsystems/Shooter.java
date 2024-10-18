package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_topShooterMotor;
    private SparkPIDController m_topPidController;
    private RelativeEncoder m_topEncoder;

    private CANSparkMax m_bottomShooterMotor;

    public Shooter(){
        m_topShooterMotor = new CANSparkMax(25, MotorType.kBrushless);
        m_topPidController = m_topShooterMotor.getPIDController();
        m_topEncoder = m_topShooterMotor.getEncoder();
        
        //m_bottomShooterMotor = new CANSparkMax(26, MotorType.kBrushless);
    }

    public void configure(){
        m_topShooterMotor.setInverted(false);
       // m_bottomShooterMotor.setInverted(false);

        m_topShooterMotor.enableVoltageCompensation(12);
        m_topShooterMotor.setIdleMode(IdleMode.kCoast);
        m_topShooterMotor.setSmartCurrentLimit(0,0);

        m_topPidController.setP(0);
        m_topPidController.setI(0);
        m_topPidController.setD(0);

        m_topPidController.setIZone(0);
        m_topPidController.setFF(0);
        m_topPidController.setOutputRange(0, 0);
        // m_bottomShooterMotor.enableVoltageCompensation(12);
        // m_bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        // m_bottomShooterMotor.setSmartCurrentLimit(60,60);
    }

    public double getVelocity(){
        return m_topEncoder.getVelocity();
    }

    public void setVelocity(double rpm){
        m_topPidController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    public void set(double percentOutput){
        m_topShooterMotor.set(percentOutput);
        // m_bottomShooterMotor.set(percentOutput);
    }

    public void stop() {
        set(0);
    }
}
