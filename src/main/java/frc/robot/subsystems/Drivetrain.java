// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {



  CANSparkMax _leftFront = new CANSparkMax(Constants.kLeftFrontMotor, MotorType.kBrushless);
  CANSparkMax _leftBack = new CANSparkMax(Constants.kLeftBackMotor, MotorType.kBrushless);

  CANSparkMax _rightFront = new CANSparkMax(Constants.kRightFrontMotor, MotorType.kBrushless);
  CANSparkMax _rightBack = new CANSparkMax(Constants.kRightBackMotor, MotorType.kBrushless);

  CANPIDController _rightPID = _rightFront.getPIDController();
  CANPIDController _leftPID = _leftFront.getPIDController();

  private final CANEncoder _leftEncoder = _leftFront.getEncoder();
  private final CANEncoder _rightEncoder = _rightFront.getEncoder();

  // The gyro sensor
  private PigeonIMU _gyro = new PigeonIMU(10);
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry _odometry;

  private final Field2d _field = new Field2d();


  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {

    _leftBack.restoreFactoryDefaults();
    _leftBack.setIdleMode(IdleMode.kBrake);

    _leftFront.restoreFactoryDefaults();
    _leftFront.setIdleMode(IdleMode.kBrake);

    _rightBack.restoreFactoryDefaults();
    _rightBack.setIdleMode(IdleMode.kBrake);

    _rightFront.restoreFactoryDefaults();
    _rightBack.setIdleMode(IdleMode.kBrake);

    _leftBack.follow(_leftFront);
    _rightBack.follow(_rightFront);


    _leftEncoder.setPositionConversionFactor(.053848); // rotations to meters
    _rightEncoder.setPositionConversionFactor(.053848); 

    _leftEncoder.setVelocityConversionFactor(.0008974); // RPM to meters per second
    _rightEncoder.setVelocityConversionFactor(.0008974);

    resetEncoders();
    _odometry = new DifferentialDriveOdometry(getHeading());

    SmartDashboard.putNumber("P Velocity", .025);

    _leftPID.setP(.001);//SmartDashboard.getNumber("P Velocity", 0));
    _leftPID.setFeedbackDevice(_leftEncoder); 


    _rightPID.setP(.001);//SmartDashboard.getNumber("P Velocity", 0));
    _rightPID.setFeedbackDevice(_rightEncoder);


    //_drive.setSafetyEnabled(false);

    SmartDashboard.putData("Field", _field);

  }

  public void tankDriveVolts(double left, double right){

    _rightFront.setVoltage(right);
    _leftFront.setVoltage(left);
  }

  public double[] getYPR(){
    double[] _ypr = new double[3];
    ErrorCode error = _gyro.getYawPitchRoll(_ypr);
    SmartDashboard.putString("IMU Error Code", error.toString());
    return _ypr;
}

public Rotation2d getHeading(){
  double ypr[] = {0,0,0};
  _gyro.getYawPitchRoll(ypr);
  return Rotation2d.fromDegrees(Math.IEEEremainder(ypr[0], 360.0d) * -1.0d);
}

  public Pose2d getPose() {
    //DriverStation.reportError("Get Pose", false);
    SmartDashboard.putBoolean("pose", !SmartDashboard.getBoolean("pose", false));
    return _odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_leftEncoder.getVelocity() * 1,
                                            _rightEncoder.getVelocity() * 1);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _odometry.resetPosition(pose, getHeading());
  }

  public void arcadeDrive(double fwd, double rot) {
  }

  public double getAverageEncoderDistance() {
    return (_leftEncoder.getPosition() + _rightEncoder.getPosition()) / 2.0;
  }

  public CANEncoder getLeftEncoder() {
    return _leftEncoder;
  }

  public CANEncoder getRightEncoder() {
    return _rightEncoder;
  }

  public void zeroHeading() {
    _gyro.setYaw(0);
  }

  public void setVelocity(double left, double right){

    SmartDashboard.putNumber("left MPS", left);
    SmartDashboard.putNumber("right MPS", right);
 
    _leftPID.setReference(-left, ControlType.kVelocity);

    _rightPID.setReference(right, ControlType.kVelocity);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    _odometry.update((getHeading()), 
                      _leftEncoder.getPosition(),
                      _rightEncoder.getPosition());

    //_leftPID.setP(SmartDashboard.getNumber("P Velocity", 0));
    //_rightPID.setP(SmartDashboard.getNumber("P Velocity", 0));

    //_field.setRobotPose(_odometry.getPoseMeters());

  }


  public void resetEncoders(){
    _rightEncoder.setPosition(0);
    _leftEncoder.setPosition(0);
  }
}
