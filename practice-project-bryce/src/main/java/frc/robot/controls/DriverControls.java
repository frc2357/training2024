package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.robot.commands.RollerRun;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.controls.util.AxisThresholdTrigger;

public class DriverControls {
    private XboxController m_controller;

    private AxisThresholdTrigger m_rightTrigger;

    private AxisThresholdTrigger m_leftTrigger;
    
    public DriverControls(XboxController controller) {
        XboxController m_controller = controller;

        m_leftTrigger = new AxisThresholdTrigger(m_controller, Axis.kLeftTrigger, 0);
        m_rightTrigger = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger,0);

        mapControls();
    }
    
    public void mapControls(){
        m_leftTrigger.whileTrue(new RollerRun(0.25));
        m_rightTrigger.whileTrue(new IntakeRun(0.25));
    }
}
