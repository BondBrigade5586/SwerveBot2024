// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.config.CTREConfigs;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  
  // CAMERA CONFIG
  private NetworkTableEntry dashboardCamera;
  public UsbCamera shooterCamera;

  // LEDS CONFIG
  private final int m_rainbowFirstPixelHue = 0;
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

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
    shooterCamera = CameraServer.startAutomaticCapture(0);
    dashboardCamera = NetworkTableInstance.getDefault().getTable("").getEntry("cameraSelection");

    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // LEDS CONFIG
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(85);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
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
    SmartDashboard.putBoolean("Is in bounds", robotContainer.armSubsystem.GetAbsolutePosition() > Constants.Arm.AmpPosition);
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    robotContainer.swerveSubsystem.zeroGyro();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // LEDS CONFIG
    if (robotContainer.intakeSubsystem.HasNote()) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setLED(i, Color.kBlue);
      }
    } 
    else {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setLED(i, Color.kGreen);
      }
    } 
    
    // Set the LEDs
    m_led.setData(m_ledBuffer);

    //////////////////////// OPERATOR CONTROLS ///////////////////////////////////

    /* SHOOTER controls */ 
    boolean shooterOn = robotContainer.GetOperatorController().getRawButton(XboxController.Button.kA.value);
    boolean reverseShooter = robotContainer.GetOperatorController().getRawButton(XboxController.Button.kRightBumper.value);
    
    if (shooterOn && !reverseShooter) {
      robotContainer.shooterSubsystem.ShooterOn();
    } else if(shooterOn && reverseShooter) {
      robotContainer.shooterSubsystem.SetBottomShooterMotorSpeed(-0.3);
      robotContainer.shooterSubsystem.SetTopShooterMotorSpeed(-0.3);
    } else {
      robotContainer.shooterSubsystem.ShooterOff();
    }

    /* INTAKE controls */ 
    double inSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kRightTrigger.value);
    double outSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kLeftTrigger.value);
    
    if ((inSpeed > Constants.Intake.triggerDeadband && !robotContainer.intakeSubsystem.HasNote()) || (shooterOn && inSpeed > Constants.Intake.triggerDeadband)) {
      robotContainer.intakeSubsystem.SetIntakeMotorSpeed(inSpeed * 0.45);
    } else if (outSpeed > Constants.Intake.triggerDeadband) {
      robotContainer.intakeSubsystem.SetIntakeMotorSpeed(-outSpeed * 0.45);
    } else {
      robotContainer.intakeSubsystem.SetIntakeMotorSpeed(0);
      if (robotContainer.intakeSubsystem.HasNote()) {
        //Change LED color
        // robotContainer.intakeSubsystem.setLED(Color.kGreen);
      }
    }     

    /* ARM controls - W/ STOP (TESTING!!) */
    // position range is 0.0836 (amp) - 0.834 (intake)
    // FIXME - FIX ABSOLUTE ENCODER!!!!!!!
    // FIXME - UPDATE TO NEW POSITION CONSTANTS < or >
    double armSpeed = robotContainer.GetOperatorController().getRawAxis(XboxController.Axis.kLeftY.value);
    boolean isInBounds = false;
    double currentArmPosition = robotContainer.armSubsystem.GetAbsolutePosition();
    SmartDashboard.putNumber("Arm Speed", armSpeed);

    if (currentArmPosition < Constants.Arm.AmpPosition
        || currentArmPosition > Constants.Arm.intakePosition
    ) {

      isInBounds = false;
      // CASE: Arm is out of bounds
      //       The arm speed is negative when the arm is moving upwards.
      //       arm 'higher' than amp or 'lower' than intake position
      //       only allow direction to return to valid range
      if (armSpeed > Constants.Shooter.stickDeadband
          && currentArmPosition < Constants.Arm.AmpPosition
      ) {
        // arm is 'higher' than amp, allow to lower arm
        robotContainer.armSubsystem.SetArmSpeed(armSpeed);
      } else if (armSpeed < -Constants.Shooter.stickDeadband
          && currentArmPosition > Constants.Arm.intakePosition
      ) {
        // arm is 'lower' than intake, allow to raise arm
        robotContainer.armSubsystem.SetArmSpeed(armSpeed);
      } else {
        robotContainer.armSubsystem.SetArmSpeed(0);
        robotContainer.armSubsystem.StopArm();
      }
    } else if (Math.abs(armSpeed) > Constants.Shooter.stickDeadband) {
      // CASE: within accepted position range (between amp & intake)
      //       set arm motor to joystick speed
      isInBounds = true;
      robotContainer.armSubsystem.SetArmSpeed(armSpeed);
    } else {
      // CASE: Arm motor at rest
      robotContainer.armSubsystem.SetArmSpeed(0);
      robotContainer.armSubsystem.StopArm();
    }
    
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
