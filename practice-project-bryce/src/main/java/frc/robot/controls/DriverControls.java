package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ParallelRun;
import frc.robot.commands.ShootParallelRun;
import frc.robot.commands.elevator.ElevatorRun;
import frc.robot.commands.intake.IntakeRunAxis;
import frc.robot.commands.pivot.SetPosition;
import frc.robot.commands.roller.RollerRun;
import frc.robot.commands.roller.ShortRollerRun;
import frc.robot.commands.shooter.ShooterRun;
import frc.robot.controls.util.AxisInterface;
import frc.robot.controls.util.AxisThresholdTrigger;

public class DriverControls {
    private XboxController m_controller;

    private AxisThresholdTrigger m_rightTrigger;

    private AxisThresholdTrigger m_leftTrigger;

    private JoystickButton m_aButton;

    private JoystickButton m_rightButton;

    private AxisInterface m_rightTriggerAxis;

    private JoystickButton m_bButton;

    private JoystickButton m_xButton;
    
    public DriverControls(XboxController controller) {
        XboxController m_controller = controller;

        m_leftTrigger = new AxisThresholdTrigger(m_controller, Axis.kLeftTrigger, 0);
        m_rightTrigger = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger,0);
        m_aButton = new JoystickButton(m_controller, Button.kA.value);
        m_rightButton = new JoystickButton(m_controller, Button.kRightBumper.value);
        m_rightTriggerAxis =  () -> {
            return m_controller.getRightTriggerAxis();
        };
        m_bButton = new JoystickButton(m_controller, Button.kB.value);
        m_xButton = new JoystickButton(m_controller, Button.kX.value);

        mapControls();
    }
    
    public void mapControls(){
        //m_leftTrigger.whileTrue(new RollerRun(0.25));
        m_rightTrigger.whileTrue(new ElevatorRun(0.25));
        //m_aButton.whileTrue(new ParallelRun(0.25));
        m_rightButton.whileTrue(new SetPosition(0.25));
        //m_bButton.whileTrue(new ShooterRun(0.25));
        //m_xButton.whileTrue(new ShootParallelRun(1));
    }
}
