package frc.robot.logging;


import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(CANcoder.class)
public class CANcoderLogger extends ClassSpecificLogger<CANcoder> {

    public CANcoderLogger() {
        super(CANcoder.class);
    }

    @Override
    protected void update(EpilogueBackend backend, CANcoder encoder) {
        backend.log("Position", encoder.getPosition().getValueAsDouble());
        backend.log("Absolute Position", encoder.getAbsolutePosition().getValueAsDouble());
        backend.log("Magnet Health", encoder.getMagnetHealth().getValue().value);
        backend.log("Device ID", encoder.getDeviceID());
    }
    
}
