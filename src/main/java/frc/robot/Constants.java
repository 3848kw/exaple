package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final double loopPeriodSecs = 0.02;
    public static boolean disableHAL = false;

    public static final class AutoConstants {
        public static final PIDConstants translationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants rotationPID = new PIDConstants(5.0, 0.0, 0.0);
    }

    public static final class SwerveConstants {
        public static final boolean chasisVelocityCorrection = false;
        public static final boolean autonomousChassisVelocityCorrection = false;
        public static final boolean ChasisSpeedsDiscretization = false;
        public static final boolean cosineCompensation = true;
        public static final boolean angularVelocityCompensation = true;
        public static final double angularVelocityCoefficient = 0.1;
    }

    public static final class TurretConstants {
        public static final int id = 40;
        public static final Translation2d turretOffset = new Translation2d(-0.104775, 0.178743);
        public static final Rotation2d rotationOffset = Rotation2d.fromDegrees(180);
        
        public static final int encoderId = 41;

        public static final double toleranceDeg = 4;
        public static final double debounceTime = 0.15;

        public static final double maxLimit = 160;
        public static final double minLimit = -160;

        public static final double leadTimeSec = 0.25;

        public static final double maxVoltageOut = 5;

        public static final class PID {
            public static final int P = 70; //was 70
            public static final int I = 0;
            public static final int D = 0;
        }
    }

    public static final class ShooterConstants {
        public static final int leaderID = 42;
        public static final int followMotorID = 43;

        public static final class FlyWheelPID {
            public static final double P = 0.12265;
            public static final double I = 0;
            public static final double D = 0;
            public static final double S = 0.28431;
            public static final double V = 0.12273;
            public static final double A = 0.023227;
        }
    }

    public static final class ClimberConstants {
        public static final int LeftMotorID = 44;
        public static final double extendPosition = 0;
        public static final double autoClimbPosition = 0;
        public static final double tolerance = 0.04;

        public static final class PID {
            public static final int P = 0;
            public static final int I = 0;
            public static final int D = 0;
            public static final int F = 0;
        }
    }

    public static final class BelthopperConstants {
        public static final int fBeltId = 9;
        public static final int bBeltId = 11;

        public static final double backBeltFeedRPM = 4000.0;

        public static final class BackBeltPID {
            public static final double P = 0.1;
            public static final double S = 0.25;
            public static final double V = 0.128;
            public static final double A = 0.0;
        }
    }

    public static final class HoodConstants {
        public static final int id = 45;
        public static final double debounceTime = 0.15;
        public static final double maxLimit = 45; // in encoder counts (rotations)
        public static final double hoodGearRatio = 103.11;
        public static final double toleranceDeg = 2.0;
        public static final boolean motorInverted = false; // Set to true if motor spins opposite
        public static final double softLimitReverse = -0.4; // reverse soft limit in degrees from home

        public static final class PID {
            public static final double P = 5;
            public static final double I = 0;
            public static final double D = 0;
        }
    }

    public static class IntakeConstants {
        public static final int angleMotorId = 46;
        public static final int intakeMotorId = 47;

        public static final double intakeMotorStatorLimit = 60.0;

        public static final double deployedPosition = 0;
        public static final double stowPosition = 7.8;
        public static final double shootingPosition = 3;

        public static final double p = 10;
        public static final double i = 0;
        public static final double d = 0;
        public static final double g = 0.4;

        public static final double motionMagicCriuseVelocity = 60;
        public static final double motionMagicCriuseAcceleration = 70;
        public static final double motionMagicJerk = 150;
         
 

    }
      
    public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}