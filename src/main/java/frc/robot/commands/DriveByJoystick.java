// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveByJoystick extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive drive;
  private final Joystick joystickR;
  private final Joystick joystickL;
  private double Rpower;
  private double Lpower;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveByJoystick(Drive subsystem,Joystick joyStickR, Joystick joystickL) {
    this.joystickR = joyStickR;
    this. joystickL = joystickL;
    drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Rpos = joystickR.getY();
    double Lpos = joystickL.getY();
    Rpower = (Math.pow(Rpos,2)>0.15)?(Math.pow(Rpos,2)*Math.signum(Rpos))/3*-1:0;
    Lpower = (Math.abs(Math.pow(Lpos,2))>0.15)?(Math.pow(Lpos,2)*Math.signum(Lpos))/3*-1:0;
    

    drive.setPower(Rpower,Lpower );
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
