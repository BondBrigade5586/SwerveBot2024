package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {

    public static final boolean fieldRelative = false;

    public static final double stickDeadband = 0.15;
    public static final double triggerDeadband = 0.03;

    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(22);//21.73); //22
    // distance between centers of right and left wheels on robot
    public static final double wheelBase = Units.inchesToMeters(22);// 24); //27
    public static final double wheelDiameter = Units.inchesToMeters(4.0); //4
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static String[] moduleNames ={"FRONT_LEFT","FRONT_RIGHT","BACK_LEFT","BACK_RIGHT"};

    // swerve drive kinematics object created based on module location
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),     // fl
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),    // fr
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),    // bl
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));  // br

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.016;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.15;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    /**
     * The maximum robot movement speed in meters per second.
     */
    public static final double maxSpeed = 4.5;
    /**
     * The maximum robot angular velocity in radians per second.
     */
    public static final double maxAngularVelocity = 11.5;
    /**
     * The slow robot movement speed in meters per second.
     */
    public static final double slowSpeed = maxSpeed / 4;
    /**
     * The slow robot angular velocity in radians per second.
     */
    public static final double slowAngularVelocity = maxAngularVelocity / 6;

    /* Neutral Modes */
    public static final CANSparkMax.IdleMode angleNeutralMode = CANSparkMax.IdleMode.kBrake;
    public static final CANSparkMax.IdleMode driveNeutralMode = CANSparkMax.IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 32;
      public static final int angleMotorID = 31;
      public static final int canCoderID = 33;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(187);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(247.15);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 42;
      public static final int angleMotorID = 41;
      public static final int canCoderID = 43;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(303.5);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(102.04);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 22;
      public static final int angleMotorID = 21;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(148.18);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(315.00);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3; //3.25
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class Shooter {
    public static final double stickDeadband = 0.15;
    public static final double triggerDeadband = 0.03;

    // PID coefficients
    public static final double kP_shooter = 6e-5; 
    public static final double kI_shooter = 0;
    public static final double kD_shooter = 0; 
    public static final double kIz_shooter = 0; 
    public static final double kFF_shooter = 0;

    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;

    public static final double onVelocity = 5000; 
    public static final double idleVelocity = onVelocity / 2;
  }

  public static final class Intake {
    public static final double triggerDeadband = 0.03;

    // PID coefficients
    public static final double kP_Intake = 0.1; 
    public static final double kI_Intake = 1e-4;
    public static final double kD_Intake = 1; 
    public static final double kIz_Intake = 0; 
    public static final double kFF_Intake = 0;
    
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;

    public static final double onVelocity = 900; 

    //Sensor variables
    public static final double sensorRange = .1524; //Six inches in meters
  }
}
