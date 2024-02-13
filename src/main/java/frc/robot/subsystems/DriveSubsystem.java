// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.ctre.VictorSPX;
import org.lasarobotics.hardware.generic.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private VictorSPX frontLeftMotor1, frontLeftMotor2;
    private VictorSPX frontRightMotor1, frontRightMotor2;
    private VictorSPX rearLeftMotor1, rearLeftMotor2;
    private VictorSP rearRightMotor1, rearRightMotor2;

    private DoubleSolenoid solenoid1, solenoid2;

    public Hardware(
        VictorSPX frontLeftMotor1, VictorSPX frontLeftMotor2,
        VictorSPX frontRightMotor1, VictorSPX frontRightMotor2,
        VictorSPX rearLeftMotor1, VictorSPX rearLeftMotor2,
        VictorSP rearRightMotor1, VictorSP rearRightMotor2,
        DoubleSolenoid solenoid1, DoubleSolenoid solenoid2) {
      this.frontLeftMotor1 = frontLeftMotor1;
      this.frontLeftMotor2 = frontLeftMotor2;
      this.frontRightMotor1 = frontRightMotor1;
      this.frontRightMotor2 = frontRightMotor2;
      this.rearLeftMotor1 = rearLeftMotor1;
      this.rearLeftMotor2 = rearLeftMotor2;
      this.rearRightMotor1 = rearRightMotor1;
      this.rearRightMotor2 = rearRightMotor2;

      this.solenoid1 = solenoid1;
      this.solenoid2 = solenoid2;
    }
  }

  // Initializes motors, drivetrain object, and navx
  private VictorSPX m_frontLeftMotor1, m_frontLeftMotor2;
  private VictorSPX m_frontRightMotor1, m_frontRightMotor2;
  private VictorSPX m_rearLeftMotor1, m_rearLeftMotor2;
  private VictorSP m_rearRightMotor1, m_rearRightMotor2;

  private DoubleSolenoid m_solenoid1, m_solenoid2;

  private boolean m_isMechanum; 

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param drivetrainHardware   Hardware devices required by drivetrain
   */
  public DriveSubsystem(Hardware drivetrainHardware) {
    // Instantiates motors and navx
    this.m_frontLeftMotor1 = drivetrainHardware.frontLeftMotor1;
    this.m_frontLeftMotor2 = drivetrainHardware.frontLeftMotor2;
    this.m_frontRightMotor1 = drivetrainHardware.frontRightMotor1;
    this.m_frontRightMotor2 = drivetrainHardware.frontRightMotor2;
    this.m_rearLeftMotor1 = drivetrainHardware.rearLeftMotor1;
    this.m_rearLeftMotor2 = drivetrainHardware.rearLeftMotor2;
    this.m_rearRightMotor1 = drivetrainHardware.rearRightMotor1;
    this.m_rearRightMotor2 = drivetrainHardware.rearRightMotor2;

    this.m_solenoid1 = drivetrainHardware.solenoid1;
    this.m_solenoid2 = drivetrainHardware.solenoid2;

    m_frontRightMotor1.setInverted(true);
    m_frontRightMotor2.setInverted(true);
    m_rearRightMotor1.setInverted(true);
    m_rearRightMotor2.setInverted(true);

    m_isMechanum = false;
    m_solenoid1.set(Value.kReverse);
    m_solenoid2.set(Value.kReverse);
  }

  /**
   * Initialize hardware devices for drive subsystem
   * 
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware drivetrainHardware = new Hardware(
      new VictorSPX(Constants.DriveHardware.FRONT_LEFT_MOTOR_1_ID),
      new VictorSPX(Constants.DriveHardware.FRONT_LEFT_MOTOR_2_ID),
      new VictorSPX(Constants.DriveHardware.FRONT_RIGHT_MOTOR_1_ID),
      new VictorSPX(Constants.DriveHardware.FRONT_RIGHT_MOTOR_2_ID),
      new VictorSPX(Constants.DriveHardware.REAR_LEFT_MOTOR_1_ID),
      new VictorSPX(Constants.DriveHardware.REAR_LEFT_MOTOR_2_ID),
      new VictorSP(Constants.DriveHardware.REAR_RIGHT_MOTOR_1_ID),
      new VictorSP(Constants.DriveHardware.REAR_RIGHT_MOTOR_2_ID),
      new DoubleSolenoid(Constants.DriveHardware.SOLENOID_1_ID, 1),
      new DoubleSolenoid(Constants.DriveHardware.SOLENOID_2_ID, 1)
    );

    return drivetrainHardware;
  }

  // Controls the robot during teleop
  private void arcadeDrive(double speed, double turn) {
    m_frontLeftMotor1.set(ControlMode.PercentOutput, speed - turn);
    m_frontLeftMotor2.set(ControlMode.PercentOutput, speed - turn);
    m_rearLeftMotor1.set(ControlMode.PercentOutput, speed - turn);
    m_rearLeftMotor2.set(ControlMode.PercentOutput, speed - turn);
    
    m_frontRightMotor1.set(ControlMode.PercentOutput, speed + turn);
    m_frontRightMotor2.set(ControlMode.PercentOutput, speed - turn);
    m_rearRightMotor1.set(speed - turn);
    m_rearRightMotor2.set(speed - turn);
  }

  private void mechanumDrive(double speed, double strafe, double turn) {

  }

  private void toggleMechanum() {
    m_isMechanum = !m_isMechanum;

    if (m_isMechanum) {
      m_solenoid1.set(Value.kForward);
      m_solenoid2.set(Value.kForward);
    }
    else {
      m_solenoid1.set(Value.kReverse);
      m_solenoid2.set(Value.kReverse);
    }
  }

  public Command driveCommand(DoubleSupplier speedRequest, DoubleSupplier turnRequest) {
    return run(
      () -> {
        if (m_isMechanum)
          // mechanumDrive(speedRequest.getAsDouble(), turnRequest.getAsDouble());
          System.out.println("Test");
        else
          arcadeDrive(speedRequest.getAsDouble(), turnRequest.getAsDouble());
      });
  }

  public Command toggleMechanumCommand() {
    return runOnce(() -> toggleMechanum());
  }

  @Override
  public void close() {
    m_frontLeftMotor1.close();
    m_frontLeftMotor2.close();
    m_frontRightMotor1.close();
    m_frontRightMotor2.close();
    m_rearLeftMotor1.close();
    m_rearLeftMotor2.close();
    m_rearRightMotor1.close();
    m_rearRightMotor2.close();
  }
}
