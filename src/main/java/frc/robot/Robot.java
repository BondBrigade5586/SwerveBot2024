// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.config.CTREConfigs;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SendableChooser<Object> autoChooser;
  public static CTREConfigs ctreConfigs;
  
  private NetworkTableEntry dashboardCamera;
  public UsbCamera shooterCamera;
  
  /**
   * The command instance for the robot's autonomous command state.
   */
  private Command autonomousCommand;
  /**
   * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm,
   * very little robot logic should actually be handled in the Robot periodic methods (other than the scheduler calls).
   * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
   */
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // autoChooser = new SendableChooser<>();

    // autoChooser.addOption("Center Position", new SequentialCommandGroup(
      
    // ));

    shooterCamera = CameraServer.startAutomaticCapture(0);
    dashboardCamera = NetworkTableInstance.getDefault().getTable("").getEntry("cameraSelection");

    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    robotContainer.intakeSubsystem.setLED(Color.kBlack);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.intakeSubsystem.setLED(Color.kBlack);
  }

  @Override
  public void disabledPeriodic() {
    // robotContainer.swerveSubsystem.zeroGyro();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // System.out.println("TELEOPINIT");
    robotContainer.swerveSubsystem.zeroGyro();
    robotContainer.intakeSubsystem.setLED(Color.kRed);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //////////////////////// TEST HARNESS CODE ///////////////////////////////////
    /////////////////////// Added sensor /////////////////////////////////////////

    // shooter controls
    boolean shooterOn = robotContainer.GetOperatorController().getRawButton(XboxController.Button.kA.value);
    boolean reverseShooter = robotContainer.GetOperatorController().getRawButton(XboxController.Button.kLeftBumper.value);
    // intake controls
    double inSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kRightTrigger.value);
    double outSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kLeftTrigger.value);
    if ((inSpeed > Constants.Intake.triggerDeadband && !robotContainer.intakeSubsystem.HasNote()) || (shooterOn && inSpeed > Constants.Intake.triggerDeadband)) {
      robotContainer.intakeSubsystem.SetIntakeMotorSpeed(inSpeed * 0.65);
      // robotContainer.intakeSubsystem.IntakeIn();
    } else if (outSpeed > Constants.Intake.triggerDeadband) {
      robotContainer.intakeSubsystem.SetIntakeMotorSpeed(-outSpeed * 05);
      // robotContainer.intakeSubsystem.IntakeOut();
    } else {
      robotContainer.intakeSubsystem.SetIntakeMotorSpeed(0);
      if (robotContainer.intakeSubsystem.HasNote()) {
        //Change LED color
        robotContainer.intakeSubsystem.setLED(Color.kGreen);
      }
      // robotContainer.intakeSubsystem.IntakeOff();
    }

    // TESTING TELEOP SHOOTER TRIGGER
    // double shooterSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kRightY.value);
    if (shooterOn && !reverseShooter) {
      // robotContainer.shooterSubsystem.SetBottomShooterMotorSpeed(0.25);
      // robotContainer.shooterSubsystem.SetTopShooterMotorSpeed(0.25);
      robotContainer.shooterSubsystem.ShooterOn();
    
    } else if(shooterOn && reverseShooter) {
      // robotContainer.shooterSubsystem.SetBottomShooterMotorSpeed(-0.3);
      // robotContainer.shooterSubsystem.SetTopShooterMotorSpeed(-0.3);
    } else {
      robotContainer.shooterSubsystem.ShooterOff();
      // robotContainer.shooterSubsystem.SetBottomShooterMotorSpeed(0);
      // robotContainer.shooterSubsystem.SetTopShooterMotorSpeed(0);
    }

    // ARM control
    double armSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kLeftY.value);
    if (Math.abs(armSpeed) > Constants.Shooter.stickDeadband) {
      // set arm motor to joystick speed
      robotContainer.armSubsystem.SetArmSpeed(armSpeed);
    } else {
      // arm motor not moving!
      // robotContainer.armSubsystem.SetArmSpeed(0);
      robotContainer.armSubsystem.StopArm();
    }
    
    //////////////////////// TEST HARNESS CODE ///////////////////////////////////
    /////////////////////// With sensor //////////////////////////////////////////
    // intake controls
    // double inSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kRightTrigger.value);
    // double outSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kLeftTrigger.value);

    // //Shooter controls
    // boolean shooterOn = robotContainer.GetOperatorController().getRawButton(XboxController.Button.kA.value);

    // // ARM control
    // double armSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kLeftY.value);

    // //Intake logic
    // if ((inSpeed > Constants.Intake.triggerDeadband && !robotContainer.intakeSubsystem.HasNote()) ||
    //       (shooterOn && inSpeed > Constants.Intake.triggerDeadband)
    // ) {
    //   // robotContainer.shooterSubsystem.SetIntakeMotorSpeed(inSpeed);
    //   robotContainer.intakeSubsystem.IntakeIn();
    // } else if (outSpeed > Constants.Intake.triggerDeadband) {
    //   // robotContainer.shooterSubsystem.SetIntakeMotorSpeed(-outSpeed);
    //   robotContainer.intakeSubsystem.IntakeOut();
    // } else {
    //   // robotContainer.shooterSubsystem.SetIntakeMotorSpeed(0);
    //   robotContainer.intakeSubsystem.IntakeOff();
    // }

    // Shooter logic
    // if (shooterOn) {
    //   robotContainer.shooterSubsystem.ShooterOn();
    // } else {
    //   robotContainer.shooterSubsystem.ShooterOff();
    // }

    // // Arm logic
    // if (Math.abs(armSpeed) > Constants.Shooter.stickDeadband) {
    //   // set arm motor to joystick speed
    //   robotContainer.armSubsystem.SetArmSpeed(armSpeed);
    // } else {
    //   // arm motor not moving!
    //   robotContainer.armSubsystem.SetArmSpeed(0);
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
