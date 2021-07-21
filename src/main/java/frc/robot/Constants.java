/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String CONFIGS_FILE = "/home/lvuser/constants.csv";

  public static final boolean DEBUG_MODE = true;

  public static final double NOMINAL_VOLTAGE = 12;

  public static final boolean TUNE_MODE = true;

  public static final String PID_ENABLE_LABEL = "PID Enable";

  public static final String TUNE_ENABLE_LABEL = "Tune Enable";

  public static final String TUNE_LAYER_LABEL = "Tune Layer";

  public static final String RESET_PID_LABEL = "Reset PID";

  public static final double GRAVITY_ACCELERATION_CONSTANT = 9.8; //meters per second

  public static final class Drivetrain {
    public static final int TALON_LEFT_PORT_LEADER = 0;
    public static final int TALON_LEFT_PORT_FOLLOWER_2 = 0;
    public static final int TALON_LEFT_PORT_FOLLOWER_3 = 0;

    public static final int TALON_RIGHT_PORT_LEADER = 0;
    public static final int TALON_RIGHT_PORT_FOLLOWER_2 = 0;
    public static final int TALON_RIGHT_PORT_FOLLOWER_3 = 0;

    public static final int LEFT_ENCODER_TALON = 0;
    public static final int RIGHT_ENCODER_TALON = 0;

    public static final int PIGEON_IMU_PORT = 0;

    public static final int MAX_CURRENT = 18;

    public static final int ENCODER_CPR = 250 * 4; // 4x encoding

    public static final double WHEEL_RADIUS = 0; // inches

    public static final double DISTANCE_PER_REVOLUTION = 2 * Math.PI * WHEEL_RADIUS;

    public static final double STEERING_SCALE = 0.8;
    public static final double THROTTLE_SCALE = 0.8;

    public static final String RESET_GYRO_LABEL = "Zero Gyro";
  }

//	public static final class Drivetrain {
//
//		public static final int SPARK_LEFT_PORT_MASTER = 10;
//		public static final int SPARK_LEFT_PORT_SLAVE_2 = 11;
//		public static final int SPARK_LEFT_PORT_SLAVE_3 = 12;
//
//		public static final int SPARK_RIGHT_PORT_MASTER = 14;
//		public static final int SPARK_RIGHT_PORT_SLAVE_2 = 15;
//		public static final int SPARK_RIGHT_PORT_SLAVE_3 = 16;
//
//		public static final int LEFT_ENCODER_SPARK = 10;
//		public static final int RIGHT_ENCODER_SPARK = 14;
//
//		public static final int PIGEON_IMU_PORT = 1;
//
//		public static final int MAX_CURRENT = 18;
//
//		public static final int ENCODER_CPR = 250 * 4; // 4x encoding
//
//		public static final double WHEEL_RADIUS = 3; // inches
//
//		public static final double DISTANCE_PER_REVOLUTION = 2 * Math.PI * WHEEL_RADIUS;
//
//		public static final double UP_RAMP_RATE = 0.4;
//		public static final double DOWN_RAMP_RATE = 0.1; //pwm deltas
//
//		public static final double STEERING_SCALE = 0.8;
//		public static final double THROTTLE_SCALE = 0.8;
//
//		public static final String RESET_GYRO_LABEL = "Zero Gyro";
//	}
}
