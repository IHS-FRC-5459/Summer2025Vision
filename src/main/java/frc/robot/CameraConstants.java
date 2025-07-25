package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class CameraConstants {
  public String kCameraName;
  public Transform3d kRobotToCam;
  public Matrix<N3, N1> kSingleTagStdDevs;
  public Matrix<N3, N1> kMultiTagStdDevs;
}
