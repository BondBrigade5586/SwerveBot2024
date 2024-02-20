package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;

    private RelativeEncoder intakeEncoder;

    private SparkPIDController intakePID;

    private Rev2mDistanceSensor distanceSensor;

    public Intake() {
        intakeMotor = new CANSparkMax(51, MotorType.kBrushless);
        
        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        intakePID = intakeMotor.getPIDController();

        // set shooter PID coefficients
        intakePID.setP(Constants.Intake.kP_Intake);
        intakePID.setI(Constants.Intake.kI_Intake);
        intakePID.setD(Constants.Intake.kD_Intake);
        intakePID.setIZone(Constants.Intake.kIz_Intake);
        intakePID.setFF(Constants.Intake.kFF_Intake);
        intakePID.setOutputRange(Constants.Intake.kMinOutput, Constants.Intake.kMaxOutput);

        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard); //The I2C port
    }

    /**
     * Set intake speed to specified power percentage
     * @param speed
     */
    public void SetIntakeMotorSpeed (double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Set PID velocity of top intake to specified RPM
     * @param velocity
     */
    public void SetIntakePIDVelocity(double velocity) {
        intakePID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Set PID velocity of intake to 1900 RPM
     */
    public void IntakeIn() {
        intakePID.setReference(Constants.Intake.onVelocity, CANSparkMax.ControlType.kVelocity);
    }
    

    /**
     * Set PID velocity of intake to negative 1900 RPM
     */
    public void IntakeOut() {
        intakePID.setReference(-Constants.Intake.onVelocity, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Set PID velocity of intake to 0 RPM
     */
    public void IntakeOff() {
        intakePID.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * 
     */
    public boolean HasNote() {
        return distanceSensor.getRange() <= Constants.Intake.sensorRange;
    }
    
}
