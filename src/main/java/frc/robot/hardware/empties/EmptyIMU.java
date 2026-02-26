package frc.robot.hardware.empties;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.BusChain;

public class EmptyIMU extends EmptyDevice implements IIMU {

	public EmptyIMU(String logPath) {
		super(logPath);
	}

	@Override
	public void setYaw(Rotation2d yaw) {}

	@Override
	public BusChain getBusChain() {
		return BusChain.ROBORIO;
	}

}
