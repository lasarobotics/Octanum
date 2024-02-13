// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.lasarobotics.hardware.ctre.VictorSPX;
import org.lasarobotics.hardware.generic.DoubleSolenoid;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
  }

  public static class DriveHardware {
    public static VictorSPX.ID FRONT_LEFT_MOTOR_1_ID = new VictorSPX.ID("DriveHardware/FrontLeft/1", 6);
    public static VictorSPX.ID FRONT_LEFT_MOTOR_2_ID = new VictorSPX.ID("DriveHardware/FrontLeft/2", 7);
    public static VictorSPX.ID FRONT_RIGHT_MOTOR_1_ID = new VictorSPX.ID("DriveHardware/FrontLeft/1", 2);
    public static VictorSPX.ID FRONT_RIGHT_MOTOR_2_ID = new VictorSPX.ID("DriveHardware/FrontLeft/2", 3);
    public static VictorSPX.ID REAR_LEFT_MOTOR_1_ID = new VictorSPX.ID("DriveHardware/FrontLeft/1", 4);
    public static VictorSPX.ID REAR_LEFT_MOTOR_2_ID = new VictorSPX.ID("DriveHardware/FrontLeft/2", 5);
    public static int REAR_RIGHT_MOTOR_1_ID = 0;
    public static int REAR_RIGHT_MOTOR_2_ID = 1;
    public static DoubleSolenoid.ID SOLENOID_1_ID = new DoubleSolenoid.ID("DriveHardware/DoubleSolenoid/1", PneumaticsModuleType.REVPH, 6, 9);
    public static DoubleSolenoid.ID SOLENOID_2_ID = new DoubleSolenoid.ID("DriveHardware/DoubleSolenoid/2", PneumaticsModuleType.REVPH, 7, 8);
  }
}
