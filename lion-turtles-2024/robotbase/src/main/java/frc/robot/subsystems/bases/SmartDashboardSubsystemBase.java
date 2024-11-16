package frc.robot.subsystems.bases;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** A base for subsystems that log values using {@link SmartDashboard}. */
public abstract class SmartDashboardSubsystemBase extends SubsystemBase {
    @Override
    public void periodic() {
        if (Constants.USE_SMART_DASHBOARD == true) {
            smartDashboardPeriodic();
        }
    }

    /** 
     * This method is only called periodically by the {@link CommandScheduler} when {@link Constants#USE_SMART_DASHBOARD} is enabled.
     * Useful for updating subsystem-specific states with the {@link SmartDashboard} under a special condition.
     * @see SmartDashboard
     * @see SubsystemBase#periodic()
    */
    public void smartDashboardPeriodic() {}
}
