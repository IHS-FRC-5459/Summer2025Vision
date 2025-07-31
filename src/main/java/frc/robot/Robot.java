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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  private Vision abiVision;
  private Vision philVision;
  private CameraConstants abiConstants;
  private CameraConstants philConstants;

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
    abiConstants = new CameraConstants();
    abiConstants.kCameraName = "Abi";
    abiConstants.kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    abiConstants.kRobotToCam =
        new Transform3d(
            new Translation3d(-0.0254, 0.0889, 0.03175),
            new Rotation3d(0, -0.34906585, -0.78539816));
    abiConstants.kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

    philConstants = new CameraConstants();
    philConstants.kCameraName = "Phil";
    philConstants.kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    philConstants.kRobotToCam =
        new Transform3d(
            new Translation3d(-0.01905, 0.3302, 0.03175),
            new Rotation3d(0, -0.52359878, 0.78539816));
    philConstants.kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

    abiVision = new Vision(abiConstants);
    philVision = new Vision(philConstants);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    abiVision.periodic();
    philVision.periodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // For localization, making a custom april tag field layout

  }

  /** This function is called periodically when disabled. */
  // PhotonPipelineResult BobRes;

  // PhotonPipelineResult PhilRes;
  // List<PhotonTrackedTarget> bobTargets;
  // List<PhotonTrackedTarget> philTargets;

  @Override
  public void disabledPeriodic() {

    // var dResults = abi.getAllUnreadResults();
    // Transform3d dFieldToCamera;
    // for (var result : dResults) {
    //   var multiTagResult = result.getMultiTagResult();
    //   if (multiTagResult.isPresent()) {
    //     dFieldToCamera = multiTagResult.get().estimatedPose.best;
    //   }
    // }
    // var pResults = Phil.getAllUnreadResults();
    // Transform3d pFieldToCamera;
    // for (var result : pResults) {
    //   var multiTagResult = result.getMultiTagResult();
    //   if (multiTagResult.isPresent()) {
    //     pFieldToCamera = multiTagResult.get().estimatedPose.best;
    //   }
    // }

    // BobRes = Bob.getLatestResult();
    // PhilRes = Phil.getLatestResult();
    // bobTargets = BobRes.getTargets();
    // PhotonTrackedTarget bTarget = PhilRes.getBestTarget();
    // philTargets = BobRes.getTargets();
    // PhotonTrackedTarget pTarget = PhilRes.getBestTarget();
    // if (bTarget != null) {
    //   Transform3d pose = bTarget.getBestCameraToTarget();

    //   Logger.recordOutput("Bob X", pose.getX());
    // }

    // boolean targetVisible = false;
    // double targetYaw = 0.0;
    // var results = camera.getAllUnreadResults();
    // if (!results.isEmpty()) {
    //   // Camera processed a new frame since last
    //   // Get the last one in the list.
    //   var result = results.get(results.size() - 1);
    //   if (result.hasTargets()) {
    //     // At least one AprilTag was seen by the camera
    //     for (var target : result.getTargets()) {
    //       if (target.getFiducialId() == 10) {
    //         // Found Tag 7, record its information
    //         targetYaw = target.getYaw();
    //         Logger.recordOutput("targetYaw", targetYaw);
    //         targetVisible = true;
    //         Logger.recordOutput("targetVisible", targetVisible);
    //       }
    //     }
    //   }
    // }
  }

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
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
