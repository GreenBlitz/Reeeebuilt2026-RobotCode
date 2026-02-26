package frc.robot.hardware.phoenix6.angleencoder;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6Device;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class CANCoderEncoder extends Phoenix6Device implements IAngleEncoder {

	private final CANcoder encoder;
	private final Phoenix6DeviceID deviceID;

	public CANCoderEncoder(String logPath, CANcoder encoder, Phoenix6DeviceID deviceID) {
		super(logPath);
		this.encoder = encoder;
		this.deviceID = deviceID;
		encoder.optimizeBusUtilization();
	}

	@Override
	public void setPosition(Rotation2d position) {
		encoder.setPosition(position.getRotations());
	}

	@Override
	public CANcoder getDevice() {
		return encoder;
	}

	public BusChain getBusChain() {
		return deviceID.busChain();
	}

}
