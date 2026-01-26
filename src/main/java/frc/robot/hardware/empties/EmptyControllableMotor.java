package frc.robot.hardware.empties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class EmptyControllableMotor implements ControllableMotor {

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), false);
	}

	@Override
	public void resetPosition(Rotation2d position) {}

	@Override
	public void applyRequest(IRequest<?> request) {}

	@Override
	public void updateSimulation() {}

	@Override
	public void setBrake(boolean brake) {}

	@Override
	public void stop() {}

	@Override
	public void setPower(double power) {}

	@Override
	public boolean isConnected() {
		return false;
	}

	@Override
	public void updateInputs(InputSignal<?>... inputSignals) {}

}
