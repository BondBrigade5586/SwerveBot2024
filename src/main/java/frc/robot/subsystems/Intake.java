package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.*;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;

    private RelativeEncoder intakeEncoder;

    private SparkPIDController intakePID;

    private Rev2mDistanceSensor distanceSensor;

  private AddressableLED robotLights;
  private AddressableLEDBuffer lightBuffer;
  private int lightStartPoint;

    public Intake() {
        intakeMotor = new CANSparkMax(51, MotorType.kBrushless);
        
        robotLights = new AddressableLED(0);
        //FIXME add the correct LED Amount
        lightBuffer = new AddressableLEDBuffer(Constants.Intake.LEDCount);
        robotLights.setLength(lightBuffer.getLength());
        lightStartPoint = 0;
        robotLights.setData(lightBuffer);
        robotLights.start();

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

        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        // Might want to change RangeProfile.kDefualt to RangeProfile.kHighSpeed
        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kDefault); //The I2C port
        distanceSensor.setAutomaticMode(true);
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

    /**
     * Set LED strip to RGB color
     * @param r
     * @param g
     * @param b
     */
    public void setLED(Color color) {
        for (int i = 0; i < lightBuffer.getLength(); i++) {
          lightBuffer.setLED(i, color);
        }
    }

    public void setRainbowLights() {
        for (int i = 0; i < lightBuffer.getLength(); i++) {
            final int hue = (lightStartPoint + (i * 180 / lightBuffer.getLength())) % 180;
            lightBuffer.setHSV(i, hue, 255, 255);
        }
        lightStartPoint += 3;
        lightStartPoint %= 180;
    }

    public void setLightData() {
        robotLights.setData(lightBuffer);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distanceSensor", distanceSensor.getRange());
        // SmartDashboard.putBoolean("distanceSensorIsValid", distanceSensor.getIsRangeValid());
        SmartDashboard.putString("distanceSensorUnits", distanceSensor.getDistanceUnits().toString());
        // robotLights.setData(lightBuffer);
    }
    
}
