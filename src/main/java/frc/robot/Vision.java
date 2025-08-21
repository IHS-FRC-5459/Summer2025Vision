/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {
  private final PhotonCamera camera;

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  /**
   * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
   *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
   */
  private AprilTagFieldLayout kTagLayout;

  CameraConstants constants;
  SwerveDrivePoseEstimator swerveEstimator;

  public Vision(CameraConstants constants, SwerveDrivePoseEstimator swerveEstimator) {
    this.constants = constants;
    this.swerveEstimator = swerveEstimator;
    if (!Robot.isSimulation()) {
      try {
        Path path = Paths.get("/home/lvuser/deploy/field.json");
        if (Files.exists(path)) {
          kTagLayout = new AprilTagFieldLayout(path);
        } else {
          System.out.println("File does not exist");
        }
      } catch (Exception e) {
        System.out.println("Error: " + e);
      }
    }
    // this.estConsumer = estConsumer;
    camera = new PhotonCamera(constants.kCameraName);

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      try {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("field.json");
        kTagLayout = new AprilTagFieldLayout(path);
        visionSim.addAprilTags(kTagLayout);
      } catch (Exception e) {
        System.out.println(e);
      }
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSim = new PhotonCameraSim(camera, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim, constants.kRobotToCam);

      cameraSim.enableDrawWireframe(true);
    }
  }

  private Pose2d latestLocation;

  public Pose2d getLatestLocation() {
    return latestLocation;
  }

  public void periodic() {
    // Gets location
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    boolean isGoodResult = true;
    for (PhotonPipelineResult result : results) {
      Optional<MultiTargetPNPResult> multiTagResult = result.getMultiTagResult();
      if (multiTagResult.isPresent()) {
        var estPose = multiTagResult.get().estimatedPose;
        Transform3d fieldToCamera = estPose.best;
        isGoodResult = estPose.ambiguity < 0.2;
        Logger.recordOutput(constants.kCameraName + "ambiguity", estPose.ambiguity);
        Translation2d transl2d = fieldToCamera.getTranslation().toTranslation2d();
        Rotation2d rot2d = fieldToCamera.getRotation().toRotation2d();
        Pose2d pose = new Pose2d(transl2d, rot2d);
        if (pose != null) {
          latestLocation = pose;
        }
      }
    }
    // Puts into swerve estimator
    if (isGoodResult && latestLocation != null) {
      swerveEstimator.addVisionMeasurement(latestLocation, Timer.getFPGATimestamp());
    }
  }
  // ----- Simulation

  public Pose3d simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
    return visionSim.getRobotPose();
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
