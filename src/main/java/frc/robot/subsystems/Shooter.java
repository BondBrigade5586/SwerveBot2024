package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // private CANSparkMax intakeMotor;
    private CANSparkMax topShooterMotor;
    private CANSparkMax bottomShooterMotor;

    private RelativeEncoder intakeEncoder;
    private RelativeEncoder topShooterEncoder;
    private RelativeEncoder bottomShooterEncoder;

    private SparkPIDController topShooterPID;
    private SparkPIDController bottomShooterPID;

    public Shooter() {
        // intakeMotor = new CANSparkMax(51, MotorType.kBrushless);
        topShooterMotor = new CANSparkMax(61, MotorType.kBrushless);
        bottomShooterMotor = new CANSparkMax(62, MotorType.kBrushless);    

        topShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        bottomShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        
        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        topShooterPID = topShooterMotor.getPIDController();
        bottomShooterPID = bottomShooterMotor.getPIDController();

        // set shooter PID coefficients
        topShooterPID.setP(Constants.Shooter.kP_shooter);
        topShooterPID.setI(Constants.Shooter.kI_shooter);
        topShooterPID.setD(Constants.Shooter.kD_shooter);
        topShooterPID.setIZone(Constants.Shooter.kIz_shooter);
        topShooterPID.setFF(Constants.Shooter.kFF_shooter);
        topShooterPID.setOutputRange(Constants.Shooter.kMinOutput, Constants.Shooter.kMaxOutput);

        bottomShooterPID.setP(Constants.Shooter.kP_shooter);
        bottomShooterPID.setI(Constants.Shooter.kI_shooter);
        bottomShooterPID.setD(Constants.Shooter.kD_shooter);
        bottomShooterPID.setIZone(Constants.Shooter.kIz_shooter);
        bottomShooterPID.setFF(Constants.Shooter.kFF_shooter);
        bottomShooterPID.setOutputRange(Constants.Shooter.kMinOutput, Constants.Shooter.kMaxOutput);
    }

    // /**
    //  * Set intake speed to specified power percentage
    //  * @param speed
    //  */
    // public void SetIntakeMotorSpeed (double speed) {
    //     intakeMotor.set(speed);
    // }

    /**
     * Set top intake speed to specified power percentage
     * @param speed
     */
    public void SetTopShooterMotorSpeed (double speed) {
        topShooterMotor.set(speed);
    }

    /**
     * Set bottom intake spped to specified power percentage
     * @param speed
     */
    public void SetBottomShooterMotorSpeed (double speed) {
        bottomShooterMotor.set(speed);
    }

    /**
     * Set PID velocity of top shooter to specified RPM
     * @param velocity
     */
    public void SetTopShooterPIDVelocity(double velocity) {
        topShooterPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Set PID velocity of bottom shooter to specified RPM
     * @param velocity
     */
    public void SetBottomShooterPIDVelocity(double velocity) {
        bottomShooterPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Set PID velocity of shooter to 3800 RPM
     */
    public void ShooterOn() {
        topShooterPID.setReference(Constants.Shooter.onVelocity, CANSparkMax.ControlType.kVelocity);
        bottomShooterPID.setReference(Constants.Shooter.onVelocity, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Set PID velocity of shooter to 0 RPM
     */
    public void ShooterOff() {
        topShooterPID.setReference(0, CANSparkMax.ControlType.kVelocity);
        bottomShooterPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Set PID velocity of shooter to RPM specified in Constants.java
     */
    public void ShooterIdle() {
        topShooterPID.setReference(Constants.Shooter.idleVelocity, CANSparkMax.ControlType.kVelocity);
        bottomShooterPID.setReference(Constants.Shooter.idleVelocity, CANSparkMax.ControlType.kVelocity);
    }
    
}
