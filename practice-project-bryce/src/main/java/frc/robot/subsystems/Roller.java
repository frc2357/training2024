package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;

public class Roller extends SubsystemBase {
    private CANSparkMax m_motor;

    public Roller(){
        m_motor = new CANSparkMax(CAN_ID.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushed);
    }

    public void set(double speed) {
        m_motor.set(speed);
    }
    
    public void stop() {
        m_motor.set(0);
    }
}
