package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TeleopShooter extends CommandBase {
  private Shooter shooterSubsystem;
  
  /**
   * A supplier for if we're currently field relative.
   * When constructing this command, we pass a lambda expression to return the current field relative mode. This supplier provides that.
   */
  private BooleanSupplier shooterOn;
  private Joystick operator;

  public TeleopShooter(
      Shooter shooterSubsystem,
      BooleanSupplier shooterOn,
      Joystick operator
      ) {
    this.shooterSubsystem = shooterSubsystem;
    this.operator = operator;
    addRequirements(shooterSubsystem);

    this.shooterOn = shooterOn;
  }

  @Override
  public void execute() {
    // shooterOn.onTrue(new InstantCommand(
    //    shooterSubsystem.ShooterOn,
    //    shooterSubsystem
    // ))
  }
}
