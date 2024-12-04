package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    public static final class CAN_ID{

    }

    public static final class SWERVE {
        public static final AngularVelocity MAX_ANGULAR_RATE = AngularVelocity.ofBaseUnits(Math.PI * 2, RotationsPerSecond);
    
        public static final Distance STATIC_FEEDFORWARD = Distance.ofBaseUnits(0.094545, Meters);
        public static final Time TIME_TO_COAST = Time.ofBaseUnits(5, Seconds);
    }

    public static final class CONTROLLER {
      public static final int DRIVE_CONTROLLER_PORT = 0;
      public static final double DRIVE_CONTROLLER_DEADBAND = 0.01;
      public static final double SWERVE_TRANSLATIONAL_DEADBAND = 0.01;
      public static final double SWERVE_ROTATIONAL_DEADBAND = 0.01;
      public static final double DRIVE_RUMBLE_INTENSITY = .5;
      public static final double DRIVE_RUMBLE_SECONDS = 2;
  
      public static final double DRIVE_TRANSLATE_INTAKE_THRESHOLD = 0.9;
    }

    public static final class CHOREO {
      public static final PIDController X_CONTROLLER = new PIDController(5, 0, 0);
      public static final PIDController Y_CONTROLLER = new PIDController(5, 0, 0);
      public static final PIDController ROTATION_CONTROLLER = new PIDController(1, 0, 0);

      /**
       * returns true if the alliance is red, false if it is blue
       */
      public static final BooleanSupplier CHOREO_AUTO_MIRROR_PATHS =
          new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
              return Robot.state.getAlliance() == Alliance.Red;
            }
          };
    }
}
