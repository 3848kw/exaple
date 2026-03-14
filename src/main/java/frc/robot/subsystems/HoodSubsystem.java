package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Constants.HoodConstants;
import frc.robot.utils.sim.PhysicsSim;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;

@Logged
public class HoodSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;

    private Rotation2d desiredAngle = new Rotation2d();
    private final Debouncer debouncer;

    private Optional<IntakeSubsystem> intake = Optional.empty();

    public HoodSubsystem() {
        motor = new SparkMax(HoodConstants.id, null);
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();

        debouncer = new Debouncer(HoodConstants.debounceTime, DebounceType.kRising);

        SparkMaxConfig configs = new SparkMaxConfig();
        configs.idleMode(IdleMode.kBrake);
        configs.inverted(HoodConstants.motorInverted);
        configs.smartCurrentLimit(60);
        configs.softLimit
            .forwardSoftLimit(encoderCountsToAngle(HoodConstants.maxLimit))
            .reverseSoftLimit(-encoderCountsToAngle(HoodConstants.softLimitReverse))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        // Configure PID for position control
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.p(HoodConstants.PID.P);
        closedLoopConfig.i(HoodConstants.PID.I);
        closedLoopConfig.d(HoodConstants.PID.D);
        closedLoopConfig.positionWrappingEnabled(false);
        configs.apply(closedLoopConfig);

        motor.configure(configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Reset encoder to zero
        encoder.setPosition(0);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
        }
    }

    private double angleToEncoderCounts(double angle) {
        // SparkMax internal encoder gives position in rotations.
        // Gear ratio: motor turns (hoodGearRatio) times for 1 full hood rotation (360 deg)
        // So: motor rotations = (angle / 360) * hoodGearRatio
        return (angle / 360.0) * HoodConstants.hoodGearRatio;
    }

    private double encoderCountsToAngle(double counts) {
        return (counts / HoodConstants.hoodGearRatio) * 360.0;
    }

    public void setAngle(Rotation2d angle) {
        double desiredDeg = angle.getDegrees();
        double clampedDeg = MathUtil.clamp(desiredDeg, 0, encoderCountsToAngle(HoodConstants.maxLimit));
        desiredAngle = Rotation2d.fromDegrees(clampedDeg);
        closedLoopController.setReference(angleToEncoderCounts(clampedDeg),ControlType.kPosition); ;
    }

    public Rotation2d getDesiredAngle() {
        return desiredAngle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoderCountsToAngle(encoder.getPosition()));
    }

    public Rotation2d getError() {
        return desiredAngle.minus(getAngle());
    }

    public Trigger atDesiredAngle() {
        return new Trigger(() ->
            debouncer.calculate(
                MathUtil.isNear(
                    desiredAngle.getDegrees(),
                    getAngle().getDegrees(),
                    HoodConstants.toleranceDeg
                )
            )
        );
    }

    public void setIntakeSubsystem(IntakeSubsystem intakeSubsystem) {
        this.intake = Optional.of(intakeSubsystem);
    }

    public boolean isAtHome() {
        return MathUtil.isNear(0.0, getAngle().getDegrees(), HoodConstants.toleranceDeg);
    }

    public Trigger safeToStowIntake() {
        return new Trigger(this::isAtHome);
    }

    public void stop() {
        motor.stopMotor();
    }

    public Command holdAngle(Supplier<Rotation2d> angle) {
        return Commands.run(() -> {
            Rotation2d targetAngle = angle.get();
            boolean targetIsHome = MathUtil.isNear(0.0, targetAngle.getDegrees(), HoodConstants.toleranceDeg);
            if (!targetIsHome && intake.isPresent() && !intake.get().isDeployed()) {
                intake.get().deploy();
                setAngle(new Rotation2d());
                return;
            }
            setAngle(targetAngle);
        }, this).withName("Hold Angle");
    }
}
