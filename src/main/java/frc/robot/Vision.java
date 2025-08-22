// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ejml.simple.SimpleMatrix;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private Camera[] cameras;

  public Vision(Camera[] cameras) {
    this.cameras = cameras;
  }

  private Pose2d fusedPose;
  private Matrix<N3, N1> fusedStdDevs;

  public void addResults(Pose2d[] poses, Matrix<N3, N1>[] stdDevsArray) {
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

  public Pose2d getPose() {
    return fusedPose;
  }

  public Matrix<N3, N1> getStdDevs() {
    return fusedStdDevs;
  }

  @Override
  public void periodic() {
    Pose2d[] poses = new Pose2d[cameras.length];
    Matrix[] stdDevs = new Matrix[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      Camera camera = cameras[i];
      camera.periodic();
      poses[i] = camera.getLatestLocation();
      stdDevs[i] = camera.getLatestStdDevs();
    }
    // This method will be called once per scheduler run
  }
}
