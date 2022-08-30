// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class MoveForward extends CommandBase {

  private final double power;

  private final double distance;

  private final Drive drive;

  private double leftStart;

  private double rightStart;

  private double moved;

  private static final double endDistance = 0.4;

  private static final double endPower = 0.7;

  /** Creates a new MoveForward. */
  public MoveForward(double power, double distance, Drive drive) {
    this.power = power;
    this.distance = distance;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftStart = drive.getLeftMeters();
    rightStart = drive.getRightMeters();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftMeters = drive.getLeftMeters();
    double rightMeters = drive.getRightMeters();
     moved = ((rightMeters - rightStart) + (leftMeters - leftStart))/2;
     double drivego = ((moved<distance - endDistance)?power : power*endPower);
    drive.setPower(drivego,drivego);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setPower(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return moved>distance ;
  }
}
