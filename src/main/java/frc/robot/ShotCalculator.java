package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class ShotCalculator {
    private final Translation2d robotToTurret;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> fieldVelSupplier;

    private final Supplier<Translation2d> targetSupplier;

    private final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Shot Calculator");
    private final DoublePublisher hoodAnglePub = table.getDoubleTopic("Hood Angle").publish();
    private final DoublePublisher shooterRpmPub = table.getDoubleTopic("Shooter RPM").publish();
    private final DoublePublisher turretAnglePub = table.getDoubleTopic("Turret Angle").publish();
    private final StructPublisher<Pose2d> lookaheadPosePub = table.getStructTopic("Lookahead Pose", Pose2d.struct).publish();
    private final DoublePublisher hubDistancePub = table.getDoubleTopic("Distance from hub").publish();
    private final DoublePublisher tofPub = table.getDoubleTopic("Time of Flight").publish();

    private ShootingParameters latest = null;

    private Double lastContinuousTurretAngleDeg = null;

    public record ShootingParameters(
        double turretFieldAngle,
        Rotation2d hoodAngle,
        double flywheelSpeed
    ) {}

    public ShotCalculator(Translation2d robotToTurret,Supplier<Pose2d> robotPoseSupplier,Supplier<ChassisSpeeds> fieldVelSupplier,Supplier<Translation2d> targetSupplier) {
        this.robotToTurret = robotToTurret;
        this.robotPoseSupplier = robotPoseSupplier;
        this.fieldVelSupplier = fieldVelSupplier;
        this.targetSupplier = targetSupplier;

        hoodAngleMap.put(1.2350742548107032, Rotation2d.fromDegrees(10));
        hoodAngleMap.put(1.3700176244296198, Rotation2d.fromDegrees(13));
        hoodAngleMap.put(1.532691954273061, Rotation2d.fromDegrees(15));
        hoodAngleMap.put(1.700048306250405, Rotation2d.fromDegrees(17));
        hoodAngleMap.put(1.8741585427045464, Rotation2d.fromDegrees(19));
        hoodAngleMap.put(2.0046301355885814, Rotation2d.fromDegrees(21));
        hoodAngleMap.put(2.116911166936387, Rotation2d.fromDegrees(21));
        hoodAngleMap.put(2.4213230290698973, Rotation2d.fromDegrees(25));
        hoodAngleMap.put(2.2520037769635177, Rotation2d.fromDegrees(27));
        hoodAngleMap.put(2.5263325169390325, Rotation2d.fromDegrees(28));
        hoodAngleMap.put(2.7148686556624475, Rotation2d.fromDegrees(30));
        hoodAngleMap.put(2.8646163783366814, Rotation2d.fromDegrees(31));
        hoodAngleMap.put(3.1211049112493665, Rotation2d.fromDegrees(31));
        hoodAngleMap.put(3.3436586102649226, Rotation2d.fromDegrees(32));
        hoodAngleMap.put(3.5695200692457125, Rotation2d.fromDegrees(33));
        hoodAngleMap.put(3.828377700342769, Rotation2d.fromDegrees(39));
        hoodAngleMap.put(4.087502324357409, Rotation2d.fromDegrees(40));
        hoodAngleMap.put(4.308528934781865, Rotation2d.fromDegrees(43));
        hoodAngleMap.put(4.461186753211828, Rotation2d.fromDegrees(44));
        hoodAngleMap.put(5.345824243875392, Rotation2d.fromDegrees(40.5));

        flywheelSpeedMap.put(1.2350742548107032, 2000.0);
        flywheelSpeedMap.put(1.3700176244296198, 2000.0);
        flywheelSpeedMap.put(1.532691954273061, 2000.0);
        flywheelSpeedMap.put(1.700048306250405, 1800.0);
        flywheelSpeedMap.put(1.8741585427045464, 1800.0);
        flywheelSpeedMap.put(2.0046301355885814, 1800.0);
        flywheelSpeedMap.put(2.116911166936387, 1850.0);
        flywheelSpeedMap.put(2.4213230290698973, 1900.0);
        flywheelSpeedMap.put(2.2520037769635177, 1900.0);
        flywheelSpeedMap.put(2.5263325169390325, 1950.0);
        flywheelSpeedMap.put(2.7148686556624475, 1900.0);
        flywheelSpeedMap.put(2.8646163783366814, 1900.0);
        flywheelSpeedMap.put(3.1211049112493665, 2000.0);
        flywheelSpeedMap.put(3.3436586102649226, 2000.0);
        flywheelSpeedMap.put(3.5695200692457125, 2100.0);
        flywheelSpeedMap.put(3.828377700342769, 2100.0);
        flywheelSpeedMap.put(4.087502324357409, 2100.0);
        flywheelSpeedMap.put(4.308528934781865, 2200.0);
        flywheelSpeedMap.put(4.461186753211828, 2200.0);
        flywheelSpeedMap.put(5.345824243875392, 2400.0);

        timeOfFlightMap.put(1.64227, 0.93);
        timeOfFlightMap.put(2.859544, 1.0);
        timeOfFlightMap.put(4.27071, 1.05);
    }

    public void clearCache() {
        latest = null;
    }

    public ShootingParameters getParameters() {
        if (latest != null) return latest;

        Translation2d target = targetSupplier.get();

        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d turretPosField = robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));
        Pose2d turretPoseField = new Pose2d(turretPosField, robotPose.getRotation());

        double turretToTargetDistance = target.getDistance(turretPoseField.getTranslation());

        // Field-relative velocity of the robot
        ChassisSpeeds v = fieldVelSupplier.get();

        // Compute turret's *field-relative* translational velocity:
        // v_turret = v_robot + omega x r
        // r is robotToTurret rotated into field frame.
        double robotAngle = robotPose.getRotation().getRadians();
        double omega = v.omegaRadiansPerSecond;

        // robotToTurret in *robot frame* is (x,y). In field, r_field = R(theta)*r_robot
        double rfx = robotToTurret.getX() * Math.cos(robotAngle) - robotToTurret.getY() * Math.sin(robotAngle);
        double rfy = robotToTurret.getX() * Math.sin(robotAngle) + robotToTurret.getY() * Math.cos(robotAngle);

        // omega x r = (-omega*rfy, omega*rfx)
        double turretVelX = v.vxMetersPerSecond + (-omega * rfy);
        double turretVelY = v.vyMetersPerSecond + ( omega * rfx);

        // Iterative time-of-flight convergence (20 iterations).
        // Each iteration re-looks up TOF from the current lookahead distance and
        // recomputes the offset, converging to a self-consistent solution.
        double tof = timeOfFlightMap.get(turretToTargetDistance);
        Pose2d lookaheadPose = turretPoseField;
        double lookaheadDistance = turretToTargetDistance;

        for (int i = 0; i < 20; i++) {
            tof = timeOfFlightMap.get(lookaheadDistance);
            lookaheadPose = new Pose2d(
                turretPoseField.getTranslation().plus(new Translation2d(turretVelX * tof, turretVelY * tof)),
                turretPoseField.getRotation()
            );
            lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Aim from lookahead position to target (FIELD angle, wrapped)
        double turretFieldAngleWrappedDeg =
            target.minus(lookaheadPose.getTranslation()).getAngle().getDegrees();

        // Unwrap to continuous degrees
        if (lastContinuousTurretAngleDeg == null) {
            lastContinuousTurretAngleDeg = turretFieldAngleWrappedDeg;
        }

        double turretFieldAngleDeg = unwrapToNearestDeg(lastContinuousTurretAngleDeg, turretFieldAngleWrappedDeg);

        lastContinuousTurretAngleDeg = turretFieldAngleDeg;

        // Hood and flywheel from lookahead distance
        Rotation2d hoodAngle = hoodAngleMap.get(lookaheadDistance);
        double flywheelSpeed = flywheelSpeedMap.get(lookaheadDistance);

        latest = new ShootingParameters(turretFieldAngleDeg, hoodAngle, flywheelSpeed);

        hoodAnglePub.set(hoodAngle.getDegrees());
        shooterRpmPub.set(flywheelSpeed);
        turretAnglePub.set(turretFieldAngleDeg);
        lookaheadPosePub.set(lookaheadPose);
        hubDistancePub.set(turretToTargetDistance);
        tofPub.set(tof);

        return latest;
    }

    private static double unwrapToNearestDeg(double referenceDeg, double candidateWrappedDeg) {
        double delta = MathUtil.inputModulus(
            candidateWrappedDeg - referenceDeg,
            -180.0,
            180.0
        );
        return referenceDeg + delta;
    }
}