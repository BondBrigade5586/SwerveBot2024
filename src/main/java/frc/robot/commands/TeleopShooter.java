package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TeleopShooter extends CommandBase {
  private Shooter shooterSubsystem;
  
  /**
   * A supplier for if we're currently field relative.
   * When constructing this command, we pass a lambda expression to return the current field relative mode. This supplier provides that.
   */
  private Joystick operator;

  public TeleopShooter(
      Shooter shooterSubsystem,
      Joystick operator
      ) {
    this.shooterSubsystem = shooterSubsystem;
    this.operator = operator;
    addRequirements(shooterSubsystem);

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
    Trigger shooterTrigger  = new Trigger (() -> {
        return operator.getRawButton(XboxController.Button.kA.value) == true;
    });

    shooterTrigger.onTrue(new InstantCommand(() -> 
        shooterSubsystem.ShooterOn(Constants.Shooter.onVelocity)
        // System.out.println("On " + shooterTrigger)
    ));

    shooterTrigger.onFalse(new InstantCommand(() -> 
        shooterSubsystem.ShooterOff()
        // System.out.println("Off " + shooterTrigger)
    ));
  }
}
