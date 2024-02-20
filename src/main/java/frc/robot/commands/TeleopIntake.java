package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TeleopIntake extends CommandBase {
  private Intake intakeSubsystem;
  
  /**
   * A supplier for if we're currently field relative.
   * When constructing this command, we pass a lambda expression to return the current field relative mode. This supplier provides that.
   */
  private Joystick operator;

  public TeleopIntake(
      Intake intakeSubsystem,
      Joystick operator
      ) {
    this.intakeSubsystem = intakeSubsystem;
    this.operator = operator;
    addRequirements(intakeSubsystem);

  }

  @Override
  public void execute() {
    // TESTING - use passed in Trigger & create commands
    // shooterOn.onTrue(new InstantCommand(() -> 
    //     shooterSubsystem.ShooterOn()
    // ));

    // shooterOn.onFalse(new InstantCommand(() -> 
    //     shooterSubsystem.ShooterOff()
    // ));

    // TESTING - create Trigger & logic all inside class (pass in joystick)
    //TODO Change to intakeTrigger
    Trigger shooterTrigger  = new Trigger (() -> {
        return operator.getRawButton(XboxController.Button.kA.value);
    });

    shooterTrigger.onTrue(new InstantCommand(() -> 
        intakeSubsystem.IntakeIn()
    ));

    shooterTrigger.onFalse(new InstantCommand(() -> 
        intakeSubsystem.IntakeOff()
    ));
  }
}
