package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;
import frc.robot.utils.sim.PhysicsSim;

@Logged
public class TurretSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final PositionVoltage positionVoltage;
    private final CANcoder encoder;

    private final Translation2d turretOffset;
    private final Supplier<Pose2d> swervePoseNow;

    // Continuous RAW desired angle (deg) in motor sensor frame
    private double desiredAngleDeg = 0.0;

    private final Debouncer debouncer;
    private boolean aimingLimited = false;

    private Optional<IntakeSubsystem> intake = Optional.empty();

    private final Alert rangeAlert;

    public TurretSubsystem(Supplier<Pose2d> swervePoseNow) {
        motor = new TalonFX(TurretConstants.id);
        encoder = new CANcoder(TurretConstants.encoderId);

        rangeAlert = new Alert("Turret outside of range", AlertType.kError);

        positionVoltage = new PositionVoltage(0);
        turretOffset = TurretConstants.turretOffset;
        this.swervePoseNow = swervePoseNow;

        debouncer = new Debouncer(TurretConstants.debounceTime, DebounceType.kRising);

        encoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = TurretConstants.PID.P;
        configs.Slot0.kI = TurretConstants.PID.I;
        configs.Slot0.kD = TurretConstants.PID.D;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configs.Voltage.PeakForwardVoltage = TurretConstants.maxVoltageOut;
        configs.Voltage.PeakReverseVoltage = -TurretConstants.maxVoltageOut;

        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = angleToEncoderCounts(TurretConstants.maxLimit);
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = angleToEncoderCounts(TurretConstants.minLimit);

        if (Robot.isReal()) {
            configs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
            configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        }

        motor.getConfigurator().apply(configs);
        motor.setNeutralMode(NeutralModeValue.Brake);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalon(motor, DCMotor.getKrakenX60(1));
            motor.getConfigurator().apply(new Slot0Configs().withKP(5).withKD(0.25));
        }
    }

    private double angleToEncoderCounts(double angleDeg) {
        return angleDeg / 360.0;
    }

    private double encoderCountsToAngle(double rotations) {
        return rotations * 360.0;
    }

    // Continuous RAW angle (deg) in motor sensor frame
    public double getAngle() {
        return encoderCountsToAngle(motor.getPosition().getValueAsDouble());
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
        motor.setControl(positionVoltage.withPosition(angleToEncoderCounts(clampedDeg)));
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
