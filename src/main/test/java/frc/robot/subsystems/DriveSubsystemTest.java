package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.drive.MAXSwerveModule;
import org.lasarobotics.drive.MAXSwerveModule.ModuleLocation;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.kauailabs.NavX2InputsAutoLogged;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
import org.lasarobotics.led.LEDStrip;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

import org.lasarobotics.hardware.generic.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem.Hardware;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)

public class DriveSubsystemTest {
    private DriveSubsystem m_driveSubsystem;
    private DriveSubsystem.Hardware m_drivetrainHardware;

    private WPI_VictorSPX m_frontLeftMasterMotor, m_frontRightMasterMotor, m_frontLeftSlaveMotor, m_frontRightSlaveMotor;
    private WPI_VictorSPX m_rearLeftMasterMotor, m_rearLeftSlaveMotor;

    private VictorSP m_rearRightMasterMotor, m_rearRightSlaveMotor;
    @BeforeEach
    public void setup() {
        m_frontLeftMasterMotor = mock(WPI_VictorSPX.class);
        m_frontRightMasterMotor = mock(WPI_VictorSPX.class);
        m_frontLeftSlaveMotor = mock(WPI_VictorSPX.class);
        m_frontRightSlaveMotor = mock(WPI_VictorSPX.class);
        m_rearLeftMasterMotor = mock(WPI_VictorSPX.class);
        m_rearRightMasterMotor = mock(VictorSP.class);
        m_rearLeftSlaveMotor = mock(WPI_VictorSPX.class);
        m_rearRightSlaveMotor = mock(VictorSP.class);

        when(m_frontLeftMasterMotor.get()).thenReturn();
        

        m_drivetrainHardware = new DriveSubsystem.Hardware(
            m_frontLeftMasterMotor,
            m_frontLeftSlaveMotor,
            m_frontRightMasterMotor,
            m_frontRightSlaveMotor,
            m_rearLeftMasterMotor,
            m_rearLeftSlaveMotor,
            m_rearRightMasterMotor,
            m_rearRightSlaveMotor);

        m_driveSubsystem = new DriveSubsystem(m_drivetrainHardware);
    }
    
    @Test
    @Order(1)
    @DisplayName("Test if robot can drive forward using tank drive")
    public void forward(){
        m_driveSubsystem.driveCommand(() -> 1.0, () -> 0.0, () -> 0.0);

        verify(m_frontLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, 1.0);
        verify(m_frontRightMasterMotor, times(1)).set(ControlMode.PercentOutput, 1.0);
        verify(m_rearLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, 1.0);
        verify(m_rearRightMasterMotor, times(1)).set(1.0);
    }

    @Test
    @Order(2)
    @DisplayName("Test if robot can drive backwards using tank drive")
    public void backward(){
        m_driveSubsystem.driveCommand(() -> -1.0, () -> 0.0, () -> 0.0);

        verify(m_frontLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, -1.0);
        verify(m_frontRightMasterMotor, times(1)).set(ControlMode.PercentOutput, -1.0);
        verify(m_rearLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, -1.0);
        verify(m_rearRightMasterMotor, times(1)).set(-1.0);
    }

    @Test
    @Order(3)
    @DisplayName("Test if robot can turn left using tank drive")
    public void left(){
        m_driveSubsystem.driveCommand(() -> 0.0, () -> -90.0, () -> 0.0);

        verify(m_frontLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, 90.0);
        verify(m_frontRightMasterMotor, times(1)).set(ControlMode.PercentOutput, 90.0);
        verify(m_rearLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, -90.0);
        verify(m_rearRightMasterMotor, times(1)).set(-90.0);
    }

    @Test
    @Order(4)
    @DisplayName("Test if robot can turn right using tank drive")
    public void right(){
        m_driveSubsystem.driveCommand(() -> 0.0, () -> 90.0, () -> 0.0);

        verify(m_frontLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, -90.0);
        verify(m_frontRightMasterMotor, times(1)).set(ControlMode.PercentOutput, -90.0);
        verify(m_rearLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, 90.0);
        verify(m_rearRightMasterMotor, times(1)).set(90.0);
    }

}
