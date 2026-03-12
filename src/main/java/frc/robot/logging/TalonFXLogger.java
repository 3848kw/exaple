package frc.robot.logging;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    protected void update(EpilogueBackend backend, TalonFX motor) {
        backend.log("Duty Cycle", motor.get());
        backend.log("Position", motor.getPosition().getValueAsDouble());
        backend.log("Velocity", motor.getVelocity().getValueAsDouble());
        backend.log("Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
        backend.log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
        backend.log("Temperature", motor.getDeviceTemp().getValueAsDouble());
        backend.log("Closed Loop Error", motor.getClosedLoopError().getValueAsDouble());
        backend.log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
        backend.log("Control Mode", motor.getControlMode().getName());
        backend.log("Device ID", motor.getDeviceID());
    }
}
