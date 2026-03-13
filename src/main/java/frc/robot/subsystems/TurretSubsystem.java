package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.utils.sim.PhysicsSim;

@Logged
public class TurretSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final CANcoder cancoder;

    private final Translation2d turretOffset;
    private final Supplier<Pose2d> swervePoseNow;

    private double desiredAngleDeg = 0.0;

    private final Debouncer debouncer;
    private boolean aimingLimited = false;

    private Optional<IntakeSubsystem> intake = Optional.empty();

    private final Alert rangeAlert;

    public TurretSubsystem(Supplier<Pose2d> swervePoseNow) {
        motor = new SparkMax(TurretConstants.id, null);
        cancoder = new CANcoder(TurretConstants.encoderId);

        rangeAlert = new Alert("Turret outside of range", AlertType.kError);

        turretOffset = TurretConstants.turretOffset;
        this.swervePoseNow = swervePoseNow;

        debouncer = new Debouncer(TurretConstants.debounceTime, DebounceType.kRising);

        SparkMaxConfig configs = new SparkMaxConfig();
        configs.idleMode(IdleMode.kBrake);
        configs.inverted(false);
        configs.smartCurrentLimit(60);
        configs.softLimit
            .forwardSoftLimit(TurretConstants.maxLimit)
            .reverseSoftLimit(TurretConstants.minLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);
            motor.configure(configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
        }

        // Zero CANcoder to absolute position on real robot
        if (Robot.isReal()) {
            cancoder.setPosition(cancoder.getAbsolutePosition().getValueAsDouble());
        }
    }

    // Continuous RAW angle (deg) from CANcoder
    public double getAngle() {
        return cancoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    // Continuous LOGICAL angle (deg) where offset defines "forward"
    public double getLogicalAngle() {
        return getAngle() + TurretConstants.rotationOffset.getDegrees();
    }

    public double getDesiredAngle() {
        return desiredAngleDeg;
    }

    public double getError() {
        return desiredAngleDeg - getAngle();
    }

    private static double unwrapNearest(double currentDeg, double wrappedTargetDeg) {
        double delta = MathUtil.inputModulus(wrappedTargetDeg - currentDeg, -180.0, 180.0);
        return currentDeg + delta;
    }

    public void setIntakeSubsystem(IntakeSubsystem intakeSubsystem) {
        this.intake = Optional.of(intakeSubsystem);
    }

    public boolean isAtHome() {
        return MathUtil.isNear(0.0, getAngle(), TurretConstants.toleranceDeg);
    }

    public Pose2d getTurretPose() {
        Pose2d swervePose = this.swervePoseNow.get();
        Translation2d turretFieldTranslation =
            swervePose.getTranslation().plus(turretOffset.rotateBy(swervePose.getRotation()));

        Rotation2d turretFieldRotation =
            swervePose.getRotation().plus(Rotation2d.fromDegrees(getLogicalAngle()));

        return new Pose2d(turretFieldTranslation, turretFieldRotation);
    }

    private static double clampToRange(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    public void setRobotRelativeAngleDeg(double desiredRawDeg) {
        double clampedDeg = MathUtil.clamp(desiredRawDeg, TurretConstants.minLimit, TurretConstants.maxLimit);
        desiredAngleDeg = clampedDeg;

        // Simple P controller using CANcoder feedback
        double error = MathUtil.inputModulus(clampedDeg - getAngle(), -180.0, 180.0);
        double output = TurretConstants.PID.P * error;
        output = MathUtil.clamp(output, -TurretConstants.maxVoltageOut, TurretConstants.maxVoltageOut);
        motor.setVoltage(output);
    }

    private void setFieldAngleDeg(double turretFieldDeg, double robotYawDeg) {
        double desiredLogicalWrapped = MathUtil.inputModulus(turretFieldDeg - robotYawDeg, -180.0, 180.0);

        double currentRaw = getAngle();
        double currentLogical = currentRaw + TurretConstants.rotationOffset.getDegrees();

        double bestRaw = Double.NaN;
        double bestCost = Double.POSITIVE_INFINITY;

        for (int k = -2; k <= 2; k++) {
            double candidateLogical = desiredLogicalWrapped + 360.0 * k;

            double candidateRaw = candidateLogical - TurretConstants.rotationOffset.getDegrees();

            if (candidateRaw < TurretConstants.minLimit || candidateRaw > TurretConstants.maxLimit) {
                continue;
            }

            double cost = Math.abs(candidateRaw - currentRaw);

            if (cost < bestCost) {
                bestCost = cost;
                bestRaw = candidateRaw;
            }
        }

        if (Double.isNaN(bestRaw)) {
            aimingLimited = true;
            double desiredLogicalContinuous = unwrapNearest(currentLogical, desiredLogicalWrapped);
            bestRaw = desiredLogicalContinuous - TurretConstants.rotationOffset.getDegrees();
            bestRaw = clampToRange(bestRaw, TurretConstants.minLimit, TurretConstants.maxLimit);
        } else {
            aimingLimited = false;
        }

        setRobotRelativeAngleDeg(bestRaw);
    }

    public Trigger atDesiredAngle() {
        return new Trigger(() ->
            debouncer.calculate(
                MathUtil.isNear(desiredAngleDeg, getAngle(), TurretConstants.toleranceDeg)
            )
        );
    }

    public Trigger aimedAtTarget() {
        return atDesiredAngle().and(() -> !aimingLimited);
    }

    public Trigger safeToStowIntake() {
        return new Trigger(this::isAtHome);
    }

    public void stop() {
        motor.stopMotor();
    }

    public Command holdRobotRelativeAngleDeg(Supplier<Double> angleDeg) {
        return Commands.run(() -> {
            if (intake.isPresent() && !intake.get().isDeployed()) {
                intake.get().deploy();
                return;
            }
            setRobotRelativeAngleDeg(angleDeg.get());
        }, this).withName("turret hold angle");
    }

    public Command holdFieldAngleDeg(Supplier<Double> fieldDeg, Supplier<Pose2d> robotPose) {
        return Commands.run(() -> {
            if (intake.isPresent() && !intake.get().isDeployed()) {
                intake.get().deploy();
                return;
            }
            double yawDeg = robotPose.get().getRotation().getDegrees();
            setFieldAngleDeg(fieldDeg.get(), yawDeg);
        }, this).withName("turret hold field angle");
    }

    @Override
    public void periodic() {
        rangeAlert.set(getAngle() < TurretConstants.minLimit || getAngle() > TurretConstants.maxLimit);
    }
}
