package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.utils.motors.TalonFxUtils;
import frc.robot.utils.sim.PhysicsSim;

@Logged
public class HoodSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final PositionVoltage positionVoltage;

    private Rotation2d desiredAngle = new Rotation2d();
    private final Debouncer debouncer;

    private Optional<IntakeSubsystem> intake = Optional.empty();

    public HoodSubsystem() {
        motor = new TalonFX(HoodConstants.id);
        positionVoltage = new PositionVoltage(0);

        debouncer = new Debouncer(HoodConstants.debounceTime, DebounceType.kRising);

        TalonFxUtils.applyDefaultConfigs(motor);
        TalonFxUtils.configSoftLimits(motor, encoderCountsToAngle(HoodConstants.maxLimit), -0.4, true);
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.Voltage.PeakForwardVoltage = HoodConstants.peakOutputVoltage;
        configs.Voltage.PeakReverseVoltage = -HoodConstants.peakOutputVoltage;
        configs.Slot0.kP = HoodConstants.PID.P;
        configs.Slot0.kI = HoodConstants.PID.I;
        configs.Slot0.kD = HoodConstants.PID.D;

        motor.getConfigurator().apply(configs);

        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.setPosition(0);

        if(Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalon(motor, DCMotor.getKrakenX60(1));
            motor.getConfigurator().apply(new Slot0Configs().withKP(5).withKD(0.25));
        }
    }

    private double angleToEncoderCounts(double angle) {
        return (HoodConstants.hoodGearRatio / 360.0) * angle;
    }

    private double encoderCountsToAngle(double counts) {
        return (counts / HoodConstants.hoodGearRatio) * 360.0;
    }

    public void setAngle(Rotation2d angle) {
        double desiredDeg = angle.getDegrees();
        double clampedDeg = MathUtil.clamp(desiredDeg, 0, encoderCountsToAngle(HoodConstants.maxLimit));
        desiredAngle = Rotation2d.fromDegrees(clampedDeg);
        motor.setControl(positionVoltage.withPosition(angleToEncoderCounts(clampedDeg)));
    }

    public Rotation2d getDesiredAngle() {
        return desiredAngle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoderCountsToAngle(motor.getPosition().getValueAsDouble()));
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
