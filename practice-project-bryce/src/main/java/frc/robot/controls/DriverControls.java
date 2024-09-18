package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ParallelRun;
import frc.robot.commands.RollerRun;
import frc.robot.commands.ShortRollerRun;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.controls.util.AxisThresholdTrigger;

public class DriverControls {
    private XboxController m_controller;

    private AxisThresholdTrigger m_rightTrigger;

    private AxisThresholdTrigger m_leftTrigger;

    private JoystickButton m_aButton;

    private JoystickButton m_rightButton;
    
    public DriverControls(XboxController controller) {
        XboxController m_controller = controller;

        m_leftTrigger = new AxisThresholdTrigger(m_controller, Axis.kLeftTrigger, 0);
        m_rightTrigger = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger,0);
        m_aButton = new JoystickButton(m_controller, Button.kA.value);
        m_rightButton = new JoystickButton(m_controller, Button.kRightBumper.value);
        
        mapControls();
    }
    
    public void mapControls(){
        m_leftTrigger.whileTrue(new RollerRun(0.25));
        m_rightTrigger.whileTrue(new IntakeRun(0.25));
        m_aButton.whileTrue(new ParallelRun(0.25));
        m_rightButton.whileTrue(new ShortRollerRun(0.25));
    }
}
