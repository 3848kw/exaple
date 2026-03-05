package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

  public static int kShooterLeader_ID = 9;
  public static int kShooterFollower_ID = 10;

  public static AngularVelocity kShooterVelocity = RPM.of(6000);

  public static Double EncoderAOffset = 0.460561;
}
