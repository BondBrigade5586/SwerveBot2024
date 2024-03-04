// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//this code is based on team frc3512 SwerveBot-2022
//other example repos: 4607, 3457

package frc.robot;

import java.nio.channels.SelectableChannel;

// import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopShooter;
import frc.robot.commands.TeleopSwerve;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  public Joystick GetOperatorController() {
    return operator;
  }

  /* Drive Controls */
  private static final int translationAxis = XboxController.Axis.kLeftY.value;
  private static final int strafeAxis = XboxController.Axis.kLeftX.value;
  private static final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  // TESTING
  // private final JoystickButton updateOdometryPose = 
  // new JoystickButton(driver, XboxController.Button.kB.value);  

  /* Subsystems */
  public final Swerve swerveSubsystem = new Swerve();
  public final Shooter shooterSubsystem = new Shooter();
  public final Arm armSubsystem = new Arm();
  public final Intake intakeSubsystem = new Intake();

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
            swerveSubsystem,
            driver,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> !robotCentric.getAsBoolean()));

    // shooterSubsystem.setDefaultCommand(
    //   new TeleopShooter(shooterSubsystem, operator)
    // );

    // Configure the button bindings
    configureButtonBindings();

    // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    autoChooser = new SendableChooser<>();
    // SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

    final JoystickButton PIDControllerButton = new JoystickButton(operator, XboxController.Button.kB.value);
    PIDControllerButton.whileTrue(new TeleopArm(armSubsystem, 0.60).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // updateOdometryPose.onTrue(new InstantCommand(() -> {
    //   swerveSubsystem.updateOdometryPose();
    // }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
