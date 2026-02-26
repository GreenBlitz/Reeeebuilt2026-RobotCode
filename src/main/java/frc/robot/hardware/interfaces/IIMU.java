package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.phoenix6.BusChain;

public interface IIMU extends IDevice {

	void setYaw(Rotation2d yaw);

	BusChain getBusChain();

}
