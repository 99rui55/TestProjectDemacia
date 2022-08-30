// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double lastError = 0;
  private double sumError = 0;
  private double kp = 0.6;
  private double ki = 0.3;
  private double kd = 0;
  private double sp;
  private double pv;
  private Drive drive;
  private double error;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PID(Drive drive, double sp) {
    this.sp = sp;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pv = (drive.getLeftMeters() + drive.getRightMeters())/2;
    error = sp-pv;
    sumError +=error;
    double p = kp*error + kd*sumError + kd*(lastError-error);
    lastError = error;
    if(p>0.4){
      p=0.4;
    }
    drive.setPower(p,p);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setPower(0,0);
    drive.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(sp)-Math.abs(pv)<=0.05;
  }
}
