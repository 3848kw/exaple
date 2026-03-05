package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ShotingOnTheFlyConstants {
  public static Transform3d robotToTurret =
      new Transform3d(
          Inches.of(-5.25).in(Meters),
          Inches.of(5.25).in(Meters),
          Inches.of(16.945).in(Meters),
          new Rotation3d(0, 0, 180));

  public static final double loopPeriodSecs = 0.01;
}
