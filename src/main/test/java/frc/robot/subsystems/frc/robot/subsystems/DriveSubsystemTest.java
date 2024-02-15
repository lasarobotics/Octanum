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
    private frc.robot.subsystems.DriveSubsystem m_driveSubsystem;
    private DriveSubsystem.Hardware m_drivetrainHardware;

    private WPI_VictorSPX m_frontLeftMasterMotor, m_frontRightMasterMotor, m_frontLeftSlaveMotor, m_frontRightSlaveMotor;
    private WPI_VictorSPX m_rearLeftMasterMotor, m_rearLeftSlaveMotor;

    private VictorSP m_rearRightMasterMotor, m_rearRightSlaveMotor;

    private DoubleSolenoid solenoid1, solenoid2;

    private MecanumDrive m_mecanumDrive;




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

        solenoid1 = mock(DoubleSolenoid.class);
        solenoid2 = mock(DoubleSolenoid.class);


        m_mecanumDrive = new MecanumDrive(m_frontLeftMasterMotor, m_rearLeftMasterMotor, m_frontRightMasterMotor, m_rearRightMasterMotor);

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
        m_driveSubsystem.driveCommand(() -> 0.0, () -> -1.0, () -> 0.0);

        verify(m_frontLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, 1.0);
        verify(m_frontRightMasterMotor, times(1)).set(ControlMode.PercentOutput, 1.0);
        verify(m_rearLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, -1.0);
        verify(m_rearRightMasterMotor, times(1)).set(-1.0);
    }

    @Test
    @Order(4)
    @DisplayName("Test if robot can turn right using tank drive")
    public void right(){
        m_driveSubsystem.driveCommand(() -> 0.0, () -> 1.0, () -> 0.0);

        verify(m_frontLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, -1.0);
        verify(m_frontRightMasterMotor, times(1)).set(ControlMode.PercentOutput, -1.0);
        verify(m_rearLeftMasterMotor, times(1)).set(ControlMode.PercentOutput, 1.0);
        verify(m_rearRightMasterMotor, times(1)).set(1.0);
    }

    @Test
    @Order(5)
    @DisplayName("Test if robot can switch drives")
    public void toggle(){
        m_driveSubsystem.toggleMechanumCommand();

        verify(solenoid1, times(1)).set(Value.kForward);
        verify(solenoid2, times(1)).set(Value.kForward);

        m_driveSubsystem.toggleMechanumCommand();

        verify(solenoid1, times(1)).set(Value.kReverse);
        verify(solenoid2, times(1)).set(Value.kReverse);


    }

    @Test
    @Order(6)
    @DisplayName("Test if robot can drive forwards in mecahnum mode")
    public void mechForwards(){
        m_driveSubsystem.toggleMechanumCommand();

        m_driveSubsystem.driveCommand(() -> 1.0, () -> 0.0, () -> 0.0);
        
        verify(m_mecanumDrive, times(1)).driveCartesian(1, 0, 0);
        m_driveSubsystem.toggleMechanumCommand();

    }

    @Test
    @Order(7)
    @DisplayName("Test if robot can drive backwards in mecahnum mode")
    public void mechBackwards(){
        
        m_driveSubsystem.toggleMechanumCommand();

        m_driveSubsystem.driveCommand(() -> -1.0, () -> 0.0, () -> 0.0);
        
        verify(m_mecanumDrive, times(1)).driveCartesian(-1, 0, 0);
        m_driveSubsystem.toggleMechanumCommand();

    }

    @Test
    @Order(8)
    @DisplayName("Test if robot can drive left in mecahnum mode")
    public void mechLeft(){
        m_driveSubsystem.toggleMechanumCommand();

        m_driveSubsystem.driveCommand(() -> 0.0, () -> 1.0, () -> 0.0);
        
        verify(m_mecanumDrive, times(1)).driveCartesian(0, 1, 0);
        m_driveSubsystem.toggleMechanumCommand();

    }
    
    @Test
    @Order(8)
    @DisplayName("Test if robot can drive right in mecahnum mode")
    public void mechRight(){
        m_driveSubsystem.toggleMechanumCommand();

        m_driveSubsystem.driveCommand(() -> 0.0, () -> -1.0, () -> 0.0);
        
        verify(m_mecanumDrive, times(1)).driveCartesian(0, -1, 0);
        m_driveSubsystem.toggleMechanumCommand();

    }

    @Test
    @Order(9)
    @DisplayName("Test if robot can drive in a random direction in mecahnum mode")
    public void mechRandom(){
        int directionx = (int) ((Math.random()*3) - 2);

        int directiony = (int) ((Math.random()*3) - 2);

        int directionz = (int) ((Math.random()*3) - 2)
        ;
        m_driveSubsystem.toggleMechanumCommand();

        m_driveSubsystem.driveCommand(() -> directionx, () -> directiony, () -> directionz);
        
        verify(m_mecanumDrive, times(1)).driveCartesian(directionx, directiony, directionz);
        m_driveSubsystem.toggleMechanumCommand();

    }


}
