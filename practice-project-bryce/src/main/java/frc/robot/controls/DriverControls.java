package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.controls.util.AxisThresholdTrigger;

public class DriverControls {
    private XboxController m_controller;

    private AxisThresholdTrigger m_rightTrigger;

    public DriverControls(XboxController controller) {
        XboxController m_controller = controller;

        m_rightTrigger = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger,0);

        mapControls();
    }
    
    public void mapControls(){
        m_rightTrigger.whileTrue(new IntakeRun(0.25));
    }
}
