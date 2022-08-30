// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class Turn extends CommandBase {
  private final Drive drive;
  private double degreesStart;

  private static final double endDegrees = 10;
  private static final double endPower = 0.8;
  private double degreesNow;
  private final double degrees;
  private final double power;
  /** Creates a new Turn. */
  public Turn(double degrees, double power, Drive drive) {
    this.degrees = degrees;
    this.power = power;
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    degreesStart = drive.getGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double temp;
    degreesNow = drive.getGyro();
    if(degreesStart+degrees>degreesNow){
      temp = (degreesStart+degrees>degreesNow + endDegrees)?power : power*endPower;
    }else{
      temp = -((degreesStart-degrees<degreesNow - endDegrees)?power : power*endPower);
    }
    drive.setPower(temp,-temp);
    SmartDashboard.putNumber("Temp: ", temp);
    System.out.println(temp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setPower(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Math.abs(degreesStart + degrees) - Math.abs(degreesNow))<=1;
  }
}
