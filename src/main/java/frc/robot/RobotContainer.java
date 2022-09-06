// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveByJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveForward;
import frc.robot.commands.PID;
import frc.robot.commands.Print;
import frc.robot.commands.Turn;
import frc.robot.commands.setvelocity;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Joystick joystickR;
  private final Joystick joystickL;

  private final Drive drive;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    joystickR = new Joystick(Constants.joystickR);
    joystickL = new Joystick(Constants.joystickL);

    
    drive=new Drive(joystickL, joystickR);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return(new InstantCommand(()->drive.setPower(0.5,0.5),drive)
    .andThen(new WaitCommand(3.5),
    new InstantCommand(()->SmartDashboard.putNumber("velocity",drive.getVelocity())),
    new InstantCommand(()->drive.setPower(0,0),drive))); 
   // setvelocity velo = new setvelocity(1, drive);
    //return velo;
  }
}
