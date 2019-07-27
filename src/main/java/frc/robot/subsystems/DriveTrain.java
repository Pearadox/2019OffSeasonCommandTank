/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TankDriveWithJoystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogGyro;

/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
public class DriveTrain extends Subsystem {
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor2;
  private DifferentialDrive m_DifferentialDrive;
  private final AnalogGyro m_gyro;

  private double m_sensitivity = 0.5d;
   
  /**
   * Create a new drive train subsystem.
   */
  public DriveTrain() {
    super();

    // Encoders may measure differently in the real world and in
    // simulation. In this example the robot moves 0.042 barleycorns
    // per tick in the real world, but the simulated encoders
    // simulate 360 tick encoders. This if statement allows for the
    // real robot to handle this difference in devices.
    m_leftMotor = new CANSparkMax(RobotMap.leftDeviceID, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(RobotMap.leftDeviceID2, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(RobotMap.rightDeviceID, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(RobotMap.rightDeviceID2, MotorType.kBrushless);

    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();

    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);

    m_DifferentialDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_gyro = new AnalogGyro(1);
  }

  /**
   * When no other command is running let the operator drive around using the
   * PS3 joystick.
   */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDriveWithJoystick());
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  public void log() {
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void drive(double left, double right) {
      m_DifferentialDrive.tankDrive(left, right);
    }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param joy The ps3 style joystick to use to drive tank style.
   */
  public void drive(Joystick joy) {  
    drive(joy.getRawAxis(1)*m_sensitivity,joy.getRawAxis(5)*m_sensitivity);
  }

  /**
   * Get the robot's heading.
   *
   * @return The robots heading in degrees.
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /**
   * Reset the robots sensors to the zero states.
   */
  public void reset() {
    m_gyro.reset();
  }

  /**
   * Get the average distance of the encoders since the last reset.
   *
   * @return The distance driven (average of left and right encoders).
   */
  public double getDistance() {
    return 0d;
  }

  /**
   * Get the distance to the obstacle.
   *
   * @return The distance to the obstacle detected by the rangefinder.
   */
  public double getDistanceToObstacle() {
    // Really meters in simulation since it's a rangefinder...
    return 0d;
  }
}
