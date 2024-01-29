package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private CANSparkMax topShooterMotor;
    private CANSparkMax bottomShooterMotor;

    private RelativeEncoder intakeEncoder;
    private RelativeEncoder topShooterEncoder;
    private RelativeEncoder bottomShooterEncoder;

    private SparkPIDController topShooterPID;
    private SparkPIDController bottomShooterPID;

    public Shooter() {
        intakeMotor = new CANSparkMax(51, MotorType.kBrushless);
        topShooterMotor = new CANSparkMax(61, MotorType.kBrushless);
        bottomShooterMotor = new CANSparkMax(62, MotorType.kBrushless);    
        
        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        topShooterPID = topShooterMotor.getPIDController();
        bottomShooterPID = bottomShooterMotor.getPIDController();

        // PID coefficients
        double kP_shooter = 0.1; 
        double kI_shooter = 1e-4;
        double kD_shooter = 1; 
        double kIz_shooter = 0; 
        double kFF_shooter = 0;

        double kMaxOutput = 1; 
        double kMinOutput = -1;

        // set shooter PID coefficients
        topShooterPID.setP(kP_shooter);
        topShooterPID.setI(kI_shooter);
        topShooterPID.setD(kD_shooter);
        topShooterPID.setIZone(kIz_shooter);
        topShooterPID.setFF(kFF_shooter);
        topShooterPID.setOutputRange(kMinOutput, kMaxOutput);

        bottomShooterPID.setP(kP_shooter);
        bottomShooterPID.setI(kI_shooter);
        bottomShooterPID.setD(kD_shooter);
        bottomShooterPID.setIZone(kIz_shooter);
        bottomShooterPID.setFF(kFF_shooter);
        bottomShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void SetIntakeMotorSpeed (double speed) {
        intakeMotor.set(speed);
    }

    public void SetTopShooterMotorSpeed (double speed) {
        topShooterMotor.set(speed);
    }

    public void SetBottomShooterMotorSpeed (double speed) {
        bottomShooterMotor.set(speed);
    }

    public void SetTopShooterPIDVelocity(double velocity) {
        topShooterPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void SetBottomShooterPIDVelocity(double velocity) {
        bottomShooterPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }
    
}
