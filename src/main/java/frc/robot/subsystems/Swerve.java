
package frc.robot.subsystems;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {
  private final AHRS gyro;
  
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] swerveModules;

  public Swerve() {
    gyro = new AHRS(SPI.Port.kMXP);
    //gyro.reset();
    // zeroGyro();
    
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

    swerveOdometry = new SwerveDriveOdometry(
      Constants.Swerve.swerveKinematics, 
      getYaw(), 
      swerveModulePositions);
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
              velocity.getX(), velocity.getY(), rotation, gyro.getRotation2d()
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
    //NavX Code
    gyro.zeroYaw();
    // gyro.reset(); //TESTING - POSSIBLE SOLUTION??
    System.out.println("ZERO GYRO");
  }

  /**
   * Gets the current yaw of the gyro, relative to when we last zeroed it.  
   * Positive Z/yaw is left.
   * @return
   */
  public Rotation2d getYaw() {
    
    //NavX Code
    Rotation2d testYaw = (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(/*180 - */((double)gyro.getYaw()))
        : Rotation2d.fromDegrees(((double)gyro.getYaw())  + 180);

        // System.out.println(testYaw);
    // // TESTING printouts
    // System.out.println("Is inverted: " + (Constants.Swerve.invertGyro));
    // System.out.println("Tranformed Yaw: " + testYaw);
    // System.out.println("Yaw: " + gyro.getYaw());
    return testYaw;
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), new SwerveModulePosition[] {
      swerveModules[0].getPosition(), 
      swerveModules[1].getPosition(), 
      swerveModules[2].getPosition(), 
      swerveModules[3].getPosition()
    });

    SmartDashboard.putNumber("Yaw", getYaw().getDegrees());
  }
}
