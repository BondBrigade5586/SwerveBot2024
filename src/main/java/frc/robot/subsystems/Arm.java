package frc.robot.subsystems;

import edu.wpi.first.math.controller.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AbsoluteEncoder.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    // Old arm code
    // private SparkPIDController leftArmPID;
    // private SparkPIDController rightArmPID;
    private CANSparkMax rightArmMotor;
    private CANSparkMax leftArmMotor;
    private DutyCycleEncoder pivotEncoder;
    private PIDController armPID;

    public Arm() {
        rightArmMotor = new CANSparkMax(54, MotorType.kBrushless);
        leftArmMotor = new CANSparkMax(53, MotorType.kBrushless);
        pivotEncoder = new DutyCycleEncoder(4);
        // Old arm code
        // leftArmPID = leftArmMotor.getPIDController();
        // rightArmPID = rightArmMotor.getPIDController();

        // leftArmPID.setP(Constants.Arm.kP_Arm);
        // leftArmPID.setI(Constants.Arm.kI_Arm);
        // leftArmPID.setD(Constants.Arm.kD_Arm);
        // leftArmPID.setIZone(Constants.Arm.kIz_Arm);
        // leftArmPID.setFF(Constants.Arm.kFF_Arm);
        // leftArmPID.setOutputRange(Constants.Arm.kMinOutput, Constants.Arm.kMaxOutput);

        // rightArmPID.setP(Constants.Arm.kP_Arm);
        // rightArmPID.setI(Constants.Arm.kI_Arm);
        // rightArmPID.setD(Constants.Arm.kD_Arm);
        // rightArmPID.setIZone(Constants.Arm.kIz_Arm);
        // rightArmPID.setFF(Constants.Arm.kFF_Arm);
        // rightArmPID.setOutputRange(Constants.Arm.kMinOutput, Constants.Arm.kMaxOutput);
        // pivotEncoder.setPositionOffset(pivotEncoder.getAbsolutePosition());
    }  

    public void SetArmPosition() {
        // leftArmPID.setReference(Constants.Arm.intakePosition, ControlType.kPosition);
        // rightArmPID.setReference(Constants.Arm.intakePosition, ControlType.kPosition);

    }
        
    /**
     * Set arm speed to specified power percentage
     * @param speed
     */
    public void SetArmSpeed (double speed) {
        rightArmMotor.set(-0.3 * speed);
        leftArmMotor.set(0.3 * speed);
    }

    public void MoveArm() {
        rightArmMotor.set(-0.3);
        leftArmMotor.set(0.3);
    }

    /**
     * Stops the arm from moving
     */
    public void StopArm() {
        // leftArmPID.setReference(leftArmMotor.getEncoder().getPosition(), ControlType.kPosition);
        // rightArmPID.setReference(rightArmMotor.getEncoder().getPosition(), ControlType.kPosition);
    }

    public double GetAbsolutePosition() {
        return pivotEncoder.getAbsolutePosition();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("encoderPosition", pivotEncoder.getAbsolutePosition());
        
    }
}
