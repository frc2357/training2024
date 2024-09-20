package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
    private CANSparkMax m_motor = new CANSparkMax(30, CANSparkLowLevel.MotorType.kBrushed);

    public void set(double speed) {
        m_motor.set(speed);
    }
    
    public void stop() {
        m_motor.set(0);
    }
}
