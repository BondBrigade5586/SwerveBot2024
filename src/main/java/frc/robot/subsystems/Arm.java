package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private CANSparkMax rightArmMotor;
    private CANSparkMax leftArmMotor;

    public Arm() {
        rightArmMotor = new CANSparkMax(54, MotorType.kBrushless);
        leftArmMotor = new CANSparkMax(53, MotorType.kBrushless);
    }  
        
    /**
     * Set arm speed to specified power percentage
     * @param speed
     */
    public void SetArmSpeed (double speed) {
        rightArmMotor.set(-0.15 * speed);
        leftArmMotor.set(0.15 * speed);
    }    
}
