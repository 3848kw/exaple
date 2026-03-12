package frc.robot.logging;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkFlex.class)
public class SparkFlexLogger extends ClassSpecificLogger<SparkFlex> {

    public SparkFlexLogger() {
        super(SparkFlex.class);
    }

    @Override
    protected void update(EpilogueBackend backend, SparkFlex motor) {
        backend.log("Duty Cycle", motor.get());
        backend.log("Position", motor.getEncoder().getPosition());
        backend.log("Velocity", motor.getEncoder().getVelocity());
        backend.log("Motor Voltage", motor.getAppliedOutput() * motor.getBusVoltage());
        backend.log("Stator Current", motor.getOutputCurrent());
        backend.log("Temperature", motor.getMotorTemperature());
        backend.log("Device ID", motor.getDeviceId());
    }
    
}
