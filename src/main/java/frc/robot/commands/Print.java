// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.plaf.synth.SynthScrollPaneUI;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class Print extends CommandBase {

  private final Drive drive;
  
  /** Creates a new Print. */
  public Print(Drive drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("LEFT POSITION: " + drive.getLeftMeters());
    System.out.println("RIGHT POSITION: " + drive.getRightMeters());
    System.out.println("VELOCITY: " + drive.getVelocity());
    System.out.println("------------------------------------------------------------");
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
