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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  /**
   * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
   *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
   */
  private AprilTagFieldLayout kTagLayout;

  CameraConstants constants;
  // SwerveDrivePoseEstimator swerveEstimator;

  public Camera(CameraConstants constants) {
    this.constants = constants;

    // this.swerveEstimator = swerveEstimator;
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
    photonEstimator =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, constants.kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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

  public Pose3d getLatestLocation() {
    return this.latestLocation;
  }

  public Matrix<N3, N1> getEstStdDevs() {
    return estStdDevs;
  }

  private Pose3d latestLocation;
  private Matrix<N3, N1> estStdDevs;

  public void periodic() {
    // MANUAL, NO MULTITAG
    /*
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    boolean isGoodResult = true;
    Matrix<N3, N1> stdDevs = VecBuilder.fill(0, 0, 0);
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
        stdDevs = stdDevsFromMulti(multiTagResult.get());
        if (pose != null) {
          latestLocation = pose;
          latestStdDevs = stdDevs;
        }
      }
    }
      */
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : camera.getAllUnreadResults()) {
      visionEst = photonEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
      if (Robot.isSimulation()) {
        visionEst.ifPresentOrElse(
            est ->
                getSimDebugField()
                    .getObject("VisionEstimation")
                    .setPose(est.estimatedPose.toPose2d()),
            () -> {
              getSimDebugField().getObject("VisionEstimation").setPoses();
            });
      }

      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            this.latestLocation = est.estimatedPose;
            this.estStdDevs = getEstimationStdDevs();
          });
    }
  }
  // Manually calculates estStdDevs
  // private Matrix<N3, N1> stdDevsFromMulti(MultiTargetPNPResult m) {
  //   PnpResult pnp = m.estimatedPose;
  //   Transform3d fieldToCamera = pnp.best;
  //   double reprojErrPx = pnp.bestReprojErr; // pixels (0 = ideal)
  //   double ambiguity = pnp.ambiguity; // 0 = unambiguous
  //   int nTags = (m.fiducialIDsUsed == null) ? 1 : Math.max(1, m.fiducialIDsUsed.size());
  //   double distance = fieldToCamera.getTranslation().getNorm(); // meters
  //   // Tunable base values — start conservative
  //   final double BASE_POS_STD = 0.5; // meters
  //   final double BASE_ANG_STD = 0.30; // radians
  //   // Scale factors (heuristic)
  //   double reprojFactor = 1.0 + (reprojErrPx / 100.0);
  //   double ambiguityFactor = 1.0 + (ambiguity * 2.0);
  //   double distanceFactor = 1.0 + (distance * 0.05);
  //   double tagCountFactor = 1.0 / Math.sqrt(nTags);
  //   double posStd = BASE_POS_STD * reprojFactor * ambiguityFactor * distanceFactor *
  // tagCountFactor;
  //   double angStd = BASE_ANG_STD * reprojFactor * ambiguityFactor * (1.0 / Math.sqrt(nTags));
  //   return VecBuilder.fill(posStd, posStd, angStd);
  // }
  // // ----- Simulation
  private Matrix<N3, N1> curStdDevs;

  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = constants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = constants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = constants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = constants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

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
