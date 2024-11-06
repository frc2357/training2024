// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OPERATOR_CONSTANTS {
    public static final int K_DRIVER_CONTROLLER_PORT = 0;
  }

  public static class CAN_ID {
    public static final int TOP_INTAKE_MOTOR_ID = 23;
    public static final int BOTTOM_INTAKE_MOTOR_ID = 24;

    public static final int TOP_SHOOTER_MOTOR_ID = 25;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 26;

    public static final int PIVOT_MOTOR_ID = 29;
  }

  public static class SHOOTER {
    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = true;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 50;
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 50;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 50;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 50;

    public static final double RAMP_RATE = .25;

    public static final double TOP_MOTOR_P = 0.0;
    public static final double TOP_MOTOR_I = 0.0;
    public static final double TOP_MOTOR_D = 0.0;
    public static final double TOP_MOTOR_FF = 0.00018;

    public static final double BOTTOM_MOTOR_P = 0.0;
    public static final double BOTTOM_MOTOR_I = 0.0;
    public static final double BOTTOM_MOTOR_D = 0.0;
    public static final double BOTTOM_MOTOR_FF = 0.00018;
  
    public static final double RPM_TOLERANCE = 100;
  }

  public static final class INTAKE {
    public static final double AXIS_MAX_SPEED = 0.8;

    public static final double DEBOUNCE_TIME_SECONDS = 0.02;

    public static final double FEED_TO_SHOOTER_TIMEOUT = 0;
    public static final double FLOOR_INTAKE_REVERSE_TIMEOUT = 0.1;

    public static final double PICKUP_SPEED_PERCENT_OUTPUT = .75;
    public static final double SLOW_PICKUP_SPEED_PERCENT_OUTPUT = .1;
    public static final double REVERSE_FEED_SPEED_PERCENT_OUTPUT = -0.2;
    public static final double FEED_SPEED_PERCENT_OUTPUT = 0.75;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = false;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 60;
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 60;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 60;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 60;
  }

  public static class PIVOT {
    public static final double MAX_PIVOT_ANGLE = 70;
    public static final double MIN_PIVOT_ANGLE = 17.5;

    public static final double END_AFFECTOR_PRELOAD_ANGLE = 68;
    public static final double INTAKE_FROM_SOURCE_ANGLE = 55;
    public static final double DEFAULT_PIVOT_ANGLE = 45; // angle of intake

    public static final boolean MOTOR_INVERTED = true;
    public static final boolean ENCODER_INVERTED = false;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final int MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double POSITION_ALLOWED_ERROR = 0.5;
    public static final boolean POSITION_PID_WRAPPING_ENABLED = false;

    public static final double AXIS_MAX_SPEED = 0.25;

    public static final double ENCODER_POSITION_CONVERSION_FACTOR = 360;
    public static final double ENCODER_VELOCITY_CONVERSION_FACTOR = 1;
    public static final double ENCODER_ZERO_OFFSET = 201.0478306 - 10;

    /* Closed loop - lines with comments were used for arm rotation 2023 and we will
     * probably need for this 
     */
    public static final double PIVOT_P = 0.0195;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;
    public static final double PIVOT_FF = 0.00045; // Barely moving: .000465
  
    public static final String PREFERENCES_ZERO_OFFSET_KEY = "PivotZeroOffset";
    public static final double ZERO_SPEED = 0.1;
    public static final double ZERO_SPEED_STOP_TOLERANCE = 0.015;
    public static final double ZERO_SPEED_INITIAL_SECONDS = 0.15;

    public static final String PIVOT_OFFSET_KEY = "Pivot Offset";
  }
}
