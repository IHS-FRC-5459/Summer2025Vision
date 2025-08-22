// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTleftLITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

/*
 *
 */
public class Robot extends LoggedRobot {
  private Vision leftVision;
  private Vision rightVision;
  private CameraConstants leftConstants;
  private CameraConstants rightConstants;
  private Pigeon2 pigeon;

  private SwerveDrivePoseEstimator swerveEstimator;

  private Field2d m_field;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
    leftConstants = new CameraConstants();
    leftConstants.kCameraName = "left";
    leftConstants.kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    leftConstants.kRobotToCam =
        new Transform3d(
            new Translation3d(-0.0254, 0.0889, 0.03175),
            new Rotation3d(0, -0.34906585, -0.78539816));
    leftConstants.kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

    rightConstants = new CameraConstants();
    rightConstants.kCameraName = "right";
    rightConstants.kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    rightConstants.kRobotToCam =
        new Transform3d(
            new Translation3d(-0.01905, 0.3302, 0.03175),
            new Rotation3d(0, -0.52359878, 0.78539816));
    rightConstants.kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

    pigeon = new Pigeon2(1, "rio");
    Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(0, 0),
          new Translation2d(0, 0),
          new Translation2d(0, 0),
          new Translation2d(0, 0)
        };
    SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d());
    Rotation2d rotation = new Rotation2d();
    SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    swerveEstimator =
        new SwerveDrivePoseEstimator(kinematics, rotation, lastModulePositions, new Pose2d());
    leftVision = new Vision(leftConstants, swerveEstimator);
    rightVision = new Vision(rightConstants, swerveEstimator);

    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);
    System.out.println("Added filed to dashboard");
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    leftVision.periodic();
    rightVision.periodic();
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = new SwerveModulePosition();
    }
    swerveEstimator.update(pigeon.getRotation2d(), positions);
    Logger.recordOutput("leftLoc", leftVision.getLatestLocation());
    Logger.recordOutput("rightLoc", rightVision.getLatestLocation());
    Logger.recordOutput("estimator", swerveEstimator.getEstimatedPosition());
    Logger.recordOutput("pigeon Rotation", pigeon.getRotation2d());

    // For glass
    m_field.setRobotPose(swerveEstimator.getEstimatedPosition());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // For localization, making a custom april tag field layout

  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  SwerveDrivePoseEstimator simPoseEstimator;

  @Override
  public void simulationInit() {
    SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d());
    Rotation2d rotation = new Rotation2d();
    SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    simPoseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rotation, lastModulePositions, new Pose2d());
  }

  private Pose2d fusedPose;
  private Matrix<N3, N1> fusedStdDevs;

  public void fusePosesWithStd(Pose2d[] poses, Matrix<N3, N1>[] stdDevsArray) {
    if (poses.length == 0 || poses.length != stdDevsArray.length) {
      throw new IllegalArgumentException("Poses and stdDevs array must be same nonzero length");
    }

    // Build info matrices (inverse covariance) for each pose
    SimpleMatrix[] infos = new SimpleMatrix[poses.length];
    for (int i = 0; i < poses.length; i++) {
      Matrix<N3, N3> cov = new Matrix<>(N3.instance, N3.instance);
      Matrix<N3, N1> stdDevs = stdDevsArray[i];

      for (int j = 0; j < 3; j++) {
        double sigma = stdDevs.get(j, 0);
        cov.set(j, j, sigma * sigma);
      }

      infos[i] =
          new SimpleMatrix(
                  new double[][] {
                    {cov.get(0, 0), cov.get(0, 1), cov.get(0, 2)},
                    {cov.get(1, 0), cov.get(1, 1), cov.get(1, 2)},
                    {cov.get(2, 0), cov.get(2, 1), cov.get(2, 2)}
                  })
              .invert();
    }

    // Weighted sum of translations (x, y) and rotation (theta)
    SimpleMatrix weightedVec = new SimpleMatrix(3, 1);
    SimpleMatrix totalInfo = new SimpleMatrix(3, 3);
    for (int i = 0; i < poses.length; i++) {
      double x = poses[i].getX();
      double y = poses[i].getY();
      double theta = poses[i].getRotation().getRadians();
      SimpleMatrix vec = new SimpleMatrix(3, 1, true, new double[] {x, y, theta});
      weightedVec = weightedVec.plus(infos[i].mult(vec));
      totalInfo = totalInfo.plus(infos[i]);
    }

    // Solve for fused translation + rotation
    SimpleMatrix fusedVec = totalInfo.invert().mult(weightedVec);
    fusedPose =
        new Pose2d(
            new Translation2d(fusedVec.get(0), fusedVec.get(1)), new Rotation2d(fusedVec.get(2)));

    // Compute fused std devs
    fusedStdDevs = new Matrix<>(N3.instance, N1.instance);
    for (int i = 0; i < 3; i++) {
      double variance = 1.0 / totalInfo.get(i, i);
      fusedStdDevs.set(i, 0, Math.sqrt(variance));
    }
  }
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    Translation2d l_transl2d = new Translation2d(2, 4);
    Rotation2d l_rot2d = new Rotation2d(40 * (Math.PI / 180));
    Pose2d l_Pose = new Pose2d(l_transl2d, l_rot2d);
    Matrix<N3, N1> l_stdDevs = VecBuilder.fill(0.3, 0.3, 0.3);

    Translation2d r_transl2d = new Translation2d(3, 5);
    Rotation2d r_rot2d = new Rotation2d(40 * (Math.PI / 180));
    Pose2d r_Pose = new Pose2d(r_transl2d, r_rot2d);
    Matrix<N3, N1> r_stdDevs = VecBuilder.fill(0.2, 0.2, 0.2);

    Rotation2d gyroRotation = new Rotation2d(40 * (Math.PI / 180));
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = new SwerveModulePosition();
    }

    simPoseEstimator.update(gyroRotation, positions);
    fusePosesWithStd(new Pose2d[] {l_Pose, r_Pose}, new Matrix[] {l_stdDevs, r_stdDevs});
    simPoseEstimator.addVisionMeasurement(fusedPose, Timer.getFPGATimestamp(), fusedStdDevs);

    Pose2d estPose = simPoseEstimator.getEstimatedPosition();

    System.out.println("est_X" + estPose.getX());
    System.out.println("est_Y" + estPose.getY());

    System.out.println("est_deg" + estPose.getRotation().getDegrees());

    m_field.setRobotPose(estPose);
  }
}
