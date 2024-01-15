//FIX ME --- MODULE POSITION!!!!!!!!!

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

// TESTING - PathPlanner
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.oi.ShuffleboardContent;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] swerveModules;

  private Field2d field;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();
    
    swerveModules =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
      swerveModules[0].getPosition(),
      swerveModules[1].getPosition(),
      swerveModules[2].getPosition(),
      swerveModules[3].getPosition()
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), swerveModulePositions);

    // FIXME - THIS MESSES UP THE ODOMETRY SOMEHOW. WE NEED TO FIX IT.
    // AutoBuilder.configureHolonomic(
    //   this::getPose, // Robot pose supplier
    //   this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //   this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //   this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //   new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //       // PIDConstants(5, 0, 0) is a sensible default; maybe we shouldn't be using the teleop values in autonomous
    //       new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD), // Translation PID constants
    //       new PIDConstants(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD), // Rotation PID constants
    //       4.5, // Max module speed, in m/s
    //       0.4, // Drive base radius in meters. Distance from robot center to furthest module.
    //       new ReplanningConfig() // Default path replanning config. See the API for the options here
    //   ),
    //   () -> {
    //     // Boolean supplier that controls when the path will be mirrored for the red alliance
    //     // This will flip the path being followed to the red side of the field.
    //     // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //         return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    //   },
    //   this // Reference to this subsystem to set requirements
    // );

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // TESTING - PathPlanner
    // Set up custom logging to add the current path to a field 2d widget
    // PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Updates the swerve drivetrain with the specified values.
   * @param velocity
   * @param rotation The angular velocity, in radians per second.
   * @param fieldRelative 
   * @param isOpenLoop 
   */
  public void drive(Translation2d velocity, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(
              velocity.getX(), velocity.getY(), rotation, getYaw()
            )
          : new ChassisSpeeds(velocity.getX(), velocity.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Resets the robot's position on the field.
   * @param pose
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), new SwerveModulePosition[] {
      swerveModules[0].getPosition(), 
      swerveModules[1].getPosition(), 
      swerveModules[2].getPosition(), 
      swerveModules[3].getPosition()
    }, pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * Resets the yaw value of the gyroscope to zero. 
   */
  public void zeroGyro() {
    gyro.setYaw(0);
  }

  /**
   * Gets the current yaw of the gyro, relative to when we last zeroed it.  
   * Positive Z/yaw is left.
   * @return
   */
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble())
        : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), new SwerveModulePosition[] {
      swerveModules[0].getPosition(), 
      swerveModules[1].getPosition(), 
      swerveModules[2].getPosition(), 
      swerveModules[3].getPosition()
    });
    field.setRobotPose(getPose());

    for (SwerveModule mod : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
