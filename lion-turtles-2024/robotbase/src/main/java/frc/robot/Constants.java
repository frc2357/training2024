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
  public static final class OPERATOR_CONSTANTS {
    public static final int K_DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class CAN_ID {
    public static final int LEFT_SHOOTER_MOTOR_ID = 0;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 0;
  }

  public static final class SHOOTER {
    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final boolean LEFT_MOTOR_INVERTED = false;

    // public static final boolean RIGHT_MOTOR_INVERTED_FROM_LEFT = true;

    public static final int LEFT_MOTOR_STALL_LIMIT_AMPS = 60;
    public static final int LEFT_MOTOR_FREE_LIMIT_AMPS = 60;

    // public static final int RIGHT_MOTOR_STALL_LIMIT_AMPS = 50;
    // public static final int RIGHT_MOTOR_FREE_LIMIT_AMPS = 50;

    public static final double RAMP_RATE = .25;

    public static final double LEFT_MOTOR_P = 0.0001;
    public static final double LEFT_MOTOR_I = 0.0000000001;
    public static final double LEFT_MOTOR_D = 0.0;
    public static final double LEFT_MOTOR_I_ZONE = 0.0;
    public static final double LEFT_MOTOR_FF = 0.00018;

    // public static final double RIGHT_MOTOR_P = 0.0;
    // public static final double RIGHT_MOTOR_I = 0.0;
    // public static final double RIGHT_MOTOR_D = 0.0;
    // public static final double RIGHT_MOTOR_I_ZONE = 0.0;
    // public static final double RIGHT_MOTOR_FF = 0.00018;
  }
}
