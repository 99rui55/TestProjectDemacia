// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveByJoystick;

public class Drive extends SubsystemBase {
  private TalonFX frontRight;
  private TalonFX frontLeft;
  private TalonFX rearRight;
  private TalonFX rearLeft;
  private PigeonIMU gyro = new PigeonIMU(Constants.GYRO);
  /** Creates a new ExampleSubsystem. */
  public Drive(Joystick left, Joystick right) {
    frontRight = new TalonFX(Constants.rightFront);
    frontRight.setInverted(true);
    frontLeft = new TalonFX(Constants.leftFront);
    frontLeft.setInverted(false);
    rearRight = new TalonFX(Constants.rightRear);
    rearRight.setInverted(true);
    rearRight.follow(frontRight);
    rearLeft = new TalonFX(Constants.leftRear);
    rearLeft.setInverted(false);
    rearLeft.follow(frontLeft);
    frontRight.config_kP(0, 0.000008);
    frontRight.config_kD(0,0);
    frontRight.config_kI(0,0);
    frontLeft.config_kP(0, 0.000008);
    frontLeft.config_kD(0,0);
    frontLeft.config_kI(0,0);
    resetEncoder();
    setDefaultCommand(new DriveByJoystick(this,left, right));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setPower(double right, double left){
    frontRight.set(ControlMode.PercentOutput, right);
    frontLeft.set(ControlMode.PercentOutput, left);
  }

  public double getGyro(){
    return gyro.getFusedHeading();
  }

  public double getLeftMeters(){
    return(frontLeft.getSelectedSensorPosition()/Constants.PULSE_PER_METER);
  }

  public double getRightMeters(){
    return(frontRight.getSelectedSensorPosition()/Constants.PULSE_PER_METER);
  }

  public double getVelocity(){
    return((frontRight.getSelectedSensorVelocity()*10/Constants.PULSE_PER_METER)+(
      frontLeft.getSelectedSensorVelocity()*10/Constants.PULSE_PER_METER))/2;
  }

  private double getFF(double vel) {
    return Constants.KS * Math.signum(vel) + vel * Constants.KV;
  }
  private double getTalonVel(double vel) {
    return vel / 10 / Constants.PULSE_PER_METER;
  }

  public void setVelocity(double lVelocity,double rVelocity){
    frontLeft.set(ControlMode.Velocity, getTalonVel(lVelocity),
     DemandType.ArbitraryFeedForward,getFF(lVelocity));

     frontRight.set(ControlMode.Velocity, getTalonVel(rVelocity),
     DemandType.ArbitraryFeedForward,getFF(rVelocity));
  }

     

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void resetEncoder(){
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }
  }
