/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
* This project is for controlling team 3238's test robot that uses
* 4 Neos/SparkMAXs for the drivetrain.
* If you need a test project that uses some other type of motors
* on the drivetrain it would be best to create a new project.
*
* Motor layout:
* Each side of the drivetrain has two motors powering the same gearbox
* so one motor on each side is set as master and the other is set to
* follow the master on its side.
* The motor IDs on the CAN network must be unique and are assigned using
* the Spark MAX client utility; change the IDs as needed in the declaration
* of the motor variables below to match what you assign using the client utility.
*
* Joystick:
* The team has used Logitech joysticks for a long time.
* On these joysticks:
* stick forward returns negative values for y axis
* stick backward returns positive values for y axis
* twist clockwise returns positive values for z axis
* twist counterclockwise returns negative values for z axis
*/
public class Robot extends TimedRobot {

  private Joystick joystickOne = new Joystick(0);

  //change motor IDs below to match assignment made in the SparkMAX client application
  private CANSparkMax _leftMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax _leftMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax _rightMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax _rightMotor2 = new CANSparkMax(2, MotorType.kBrushless);

  private static final double THROTTLE_DEADBAND = 0.02; //may want to change depending on whether using normal() or exponentialDrive()
  private static final double TWIST_DEADBAND = 0.02; //may want to change depending on whether using normal() or exponentialDrive()
  private static final int DRIVE_MOTOR_CURRENT_LIMIT = 60; //amps
  private static final double THROTTLE_MULTIPLIER = 0.9; //scale down the joystick throttle, should be between 0 and 1
  private static final double TWIST_MULTIPLIER = 0.5; //scale down the joystick twist, should be between 0 and 1

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //web cams will not display in driver station without next line
    CameraServer.getInstance().startAutomaticCapture();

    configureMotors();
  }

  private void configureMotors()
  {
    //set to factory defaults so have a known starting configuration
    _rightMotor1.restoreFactoryDefaults();
    _rightMotor2.restoreFactoryDefaults();
    _leftMotor1.restoreFactoryDefaults();
    _leftMotor2.restoreFactoryDefaults();

    _leftMotor1.set(0);
    _leftMotor2.set(0);
    _rightMotor1.set(0);
    _rightMotor2.set(0);

    _leftMotor1.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    _leftMotor2.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    _rightMotor1.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    _rightMotor2.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    _leftMotor1.setInverted(true);
    _leftMotor2.setInverted(true);

    _rightMotor1.setInverted(false);
    _rightMotor2.setInverted(false);

    _leftMotor2.follow(_leftMotor1);
    _rightMotor2.follow(_rightMotor1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //normalDrive();
    exponentialDrive();
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Take raw values from joystick and scale linearly
   */
  private void normalDrive()
  {
    double throttle = joystickOne.getRawAxis(1) * THROTTLE_MULTIPLIER; //axis 1 = y axis = stick forward/back
    double twist = joystickOne.getRawAxis(2) * TWIST_MULTIPLIER; //axis 2 = z axis = twist

    if (Math.abs(throttle) < THROTTLE_DEADBAND)
    {
      throttle = 0.0;
    }

    if (Math.abs(twist) < TWIST_DEADBAND)
    {
      twist = 0.0;
    }

    //right motors go forward on negative values normally,
    //forward joystick produces negative values so right motors
    //should not be inverted and left motors should be inverted,
    //set in configureMotors(). Clockwise twist on joystick
    //produces positive values and means turn right so more power
    //to left motors and less power to right motors, so right
    //motors get twist value added because positive twist makes
    //right motor go backwards which is correct for turning right.
    double rightPower = throttle + twist;
    double leftPower = throttle - twist;

    SmartDashboard.putString("Throttle", Double.toString(throttle));
    SmartDashboard.putString("Twist", Double.toString(twist));
    SmartDashboard.putString("Right Power", Double.toString(rightPower));
    SmartDashboard.putString("Left Power", Double.toString(leftPower));

    //set power to masters,  don't have to set power to followers, happens automatically
    //as part of being set as follower
    _rightMotor1.set(rightPower);
    _leftMotor1.set(leftPower);
  }

  /**
   * Take raw inputs from joystick and square them (but retain original sign)
   * to get finer control at smaller joystick movements
   */
  private void exponentialDrive()
  {
    //square the joystick inputs, retain original sign, and scale
    double rawStick = joystickOne.getRawAxis(1);
    double rawTwist = joystickOne.getRawAxis(2);
    double throttle = rawStick * Math.abs(rawStick) * THROTTLE_MULTIPLIER; //axis 1 = y axis = stick forward/back
    double twist = rawTwist * Math.abs(rawTwist) * TWIST_MULTIPLIER; //axis 2 = z axis = twist

    if (Math.abs(throttle) < THROTTLE_DEADBAND)
    {
      throttle = 0.0;
    }

    if (Math.abs(twist) < TWIST_DEADBAND)
    {
      twist = 0.0;
    }

    //right motors go forward on negative values normally,
    //forward joystick produces negative values so right motors
    //should not be inverted and left motors should be inverted,
    //set in configureMotors(). Clockwise twist on joystick
    //produces positive values and means turn right so more power
    //to left motors and less power to right motors, so right
    //motors get twist value added because positive twist makes
    //right motor go backwards which is correct for turning right.
    double rightPower = throttle + twist;
    double leftPower = throttle - twist;

    SmartDashboard.putString("Throttle", Double.toString(throttle));
    SmartDashboard.putString("Twist", Double.toString(twist));
    SmartDashboard.putString("Right Power", Double.toString(rightPower));
    SmartDashboard.putString("Left Power", Double.toString(leftPower));

    //set power to masters,  don't have to set power to followers, happens automatically
    //as part of being set as follower
    _rightMotor1.set(rightPower);
    _leftMotor1.set(leftPower);
  }

}
