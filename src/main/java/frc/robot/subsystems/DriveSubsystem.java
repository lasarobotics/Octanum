// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

// import org.lasarobotics.hardware.ctre.VictorSPX;
import org.lasarobotics.hardware.generic.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private WPI_VictorSPX frontLeftMasterMotor, frontLeftSlaveMotor;
    private WPI_VictorSPX frontRightMasterMotor, frontRightSlaveMotor;
    private WPI_VictorSPX rearLeftMasterMotor, rearLeftSlaveMotor;
    private VictorSP rearRightMasterMotor, rearRightSlaveMotor;

    private DoubleSolenoid solenoid1, solenoid2;

    public Hardware(
        WPI_VictorSPX frontLeftMasterMotor, WPI_VictorSPX frontLeftSlaveMotor,
        WPI_VictorSPX frontRightMasterMotor, WPI_VictorSPX frontRightSlaveMotor,
        WPI_VictorSPX rearLeftMasterMotor, WPI_VictorSPX rearLeftSlaveMotor,
        VictorSP rearRightMasterMotor, VictorSP rearRightSlaveMotor,
        DoubleSolenoid solenoid1, DoubleSolenoid solenoid2) {
      this.frontLeftMasterMotor = frontLeftMasterMotor;
      this.frontLeftSlaveMotor = frontLeftSlaveMotor;
      this.frontRightMasterMotor = frontRightMasterMotor;
      this.frontRightSlaveMotor = frontRightSlaveMotor;
      this.rearLeftMasterMotor = rearLeftMasterMotor;
      this.rearLeftSlaveMotor = rearLeftSlaveMotor;
      this.rearRightMasterMotor = rearRightMasterMotor;
      this.rearRightSlaveMotor = rearRightSlaveMotor;

      this.solenoid1 = solenoid1;
      this.solenoid2 = solenoid2;
    }
  }

  // Have to use vanilla VictorSPX controllers for now because of lack of follow method in PurpleLib
  private WPI_VictorSPX m_frontLeftMasterMotor, m_frontLeftSlaveMotor;
  private WPI_VictorSPX m_frontRightMasterMotor, m_frontRightSlaveMotor;
  private WPI_VictorSPX m_rearLeftMasterMotor, m_rearLeftSlaveMotor;
  private VictorSP m_rearRightMasterMotor, m_rearRightSlaveMotor;

  private boolean m_isMechanum;
  private MecanumDrive m_mecanumDrive;

  private DoubleSolenoid m_solenoid1, m_solenoid2;


  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param drivetrainHardware   Hardware devices required by drivetrain
   */
  public DriveSubsystem(Hardware drivetrainHardware) {
    this.m_frontLeftMasterMotor = drivetrainHardware.frontLeftMasterMotor;
    this.m_frontLeftSlaveMotor = drivetrainHardware.frontLeftSlaveMotor;
    this.m_frontRightMasterMotor = drivetrainHardware.frontRightMasterMotor;
    this.m_frontRightSlaveMotor = drivetrainHardware.frontRightSlaveMotor;
    this.m_rearLeftMasterMotor = drivetrainHardware.rearLeftMasterMotor;
    this.m_rearLeftSlaveMotor = drivetrainHardware.rearLeftSlaveMotor;
    this.m_rearRightMasterMotor = drivetrainHardware.rearRightMasterMotor;
    this.m_rearRightSlaveMotor = drivetrainHardware.rearRightSlaveMotor;

    this.m_frontLeftMasterMotor.setNeutralMode(NeutralMode.Brake);
    this.m_frontLeftSlaveMotor.setNeutralMode(NeutralMode.Brake);
    this.m_frontRightMasterMotor.setNeutralMode(NeutralMode.Brake);
    this.m_frontRightSlaveMotor.setNeutralMode(NeutralMode.Brake);
    this.m_rearLeftMasterMotor.setNeutralMode(NeutralMode.Brake);
    this.m_rearLeftSlaveMotor.setNeutralMode(NeutralMode.Brake);

    this.m_solenoid1 = drivetrainHardware.solenoid1;
    this.m_solenoid2 = drivetrainHardware.solenoid2;

    m_frontRightMasterMotor.setInverted(true);
    m_rearRightMasterMotor.setInverted(true);

    m_frontLeftSlaveMotor.follow(m_frontLeftMasterMotor);
    m_frontRightSlaveMotor.follow(m_frontRightMasterMotor);
    m_rearLeftSlaveMotor.follow(m_rearLeftMasterMotor);
    m_rearRightMasterMotor.addFollower(m_rearRightSlaveMotor); // Because VictorSP is PWM only :(

    m_isMechanum = false;
    m_mecanumDrive = new MecanumDrive(m_frontLeftMasterMotor, m_rearLeftMasterMotor, m_frontRightMasterMotor, m_rearRightMasterMotor);
    m_solenoid1.set(Value.kReverse);
    m_solenoid2.set(Value.kReverse);
  }

  /**
   * Initialize hardware devices for drive subsystem
   * 
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    // Have to use vanilla VictorSPX controllers for now because of lack of follow method in PurpleLib
    Hardware drivetrainHardware = new Hardware(
      new WPI_VictorSPX(Constants.DriveHardware.FRONT_LEFT_MASTER_MOTOR_ID.deviceID),
      new WPI_VictorSPX(Constants.DriveHardware.FRONT_LEFT_SLAVE_MOTOR_ID.deviceID),
      new WPI_VictorSPX(Constants.DriveHardware.FRONT_RIGHT_MASTER_MOTOR_ID.deviceID),
      new WPI_VictorSPX(Constants.DriveHardware.FRONT_RIGHT_SLAVE_MOTOR_ID.deviceID),
      new WPI_VictorSPX(Constants.DriveHardware.REAR_LEFT_MASTER_MOTOR_ID.deviceID),
      new WPI_VictorSPX(Constants.DriveHardware.REAR_LEFT_SLAVE_MOTOR_ID.deviceID),
      new VictorSP(Constants.DriveHardware.REAR_RIGHT_MASTER_MOTOR_ID),
      new VictorSP(Constants.DriveHardware.REAR_RIGHT_SLAVE_MOTOR_ID),
      new DoubleSolenoid(Constants.DriveHardware.SOLENOID_1_ID, 1),
      new DoubleSolenoid(Constants.DriveHardware.SOLENOID_2_ID, 1)
    );

    return drivetrainHardware;
  }

  // Controls the robot during teleop
  private void arcadeDrive(double speed, double turn) {
    m_frontLeftMasterMotor.set(ControlMode.PercentOutput, speed + turn);
    m_rearLeftMasterMotor.set(ControlMode.PercentOutput, speed + turn);
    m_frontRightMasterMotor.set(ControlMode.PercentOutput, speed - turn);
    m_rearRightMasterMotor.set(speed - turn);
  }

  private void mechanumDrive(double speed, double strafe, double turn) {
    m_mecanumDrive.driveCartesian(speed, -strafe, turn);
  }

  private void toggleMechanum() {
    m_isMechanum = !m_isMechanum;

    System.out.println("flipped to: " + m_isMechanum);

    if (m_isMechanum) {
      m_solenoid1.set(Value.kForward);
      m_solenoid2.set(Value.kForward);
    }
    else {
      m_solenoid1.set(Value.kReverse);
      m_solenoid2.set(Value.kReverse);
    }
  }

  public Command driveCommand(DoubleSupplier speedRequest, DoubleSupplier strafeRequest, DoubleSupplier turnRequest) {
    return run(
      () -> {
        if (m_isMechanum) {
          mechanumDrive(speedRequest.getAsDouble(), strafeRequest.getAsDouble(), turnRequest.getAsDouble());
        }
        else {
          mechanumDrive(0, 0, 0);
          arcadeDrive(speedRequest.getAsDouble(), turnRequest.getAsDouble());
        }
      });
  }

  public Command runRightFront() {
    return runEnd(() -> m_rearRightMasterMotor.set(1),
    () -> m_rearRightMasterMotor.stopMotor());
  }

  public Command runLeftRear() {
    return runEnd(() -> m_rearLeftMasterMotor.set(1),
    () -> m_rearLeftMasterMotor.stopMotor());
  }

  public Command toggleMechanumCommand() {
    return runOnce(() -> toggleMechanum());
  }

  @Override
  public void close() {
    m_frontLeftMasterMotor.close();
    m_frontLeftSlaveMotor.close();
    m_frontRightMasterMotor.close();
    m_frontRightSlaveMotor.close();
    m_rearLeftMasterMotor.close();
    m_rearLeftSlaveMotor.close();
    m_rearRightMasterMotor.close();
    m_rearRightSlaveMotor.close();
  }
}
