package frc.robot.hardware.empties;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.TimedValue;
import org.littletonrobotics.junction.LogTable;

public class EmptyAngleSignal implements InputSignal<Rotation2d> {

	@Override
	public String getName() {
		return "";
	}

	@Override
	public TimedValue<Rotation2d> getLatestTimedValue() {
		return new TimedValue<>(new Rotation2d(), 0);
	}

	@Override
	public Rotation2d getLatestValue() {
		return new Rotation2d();
	}

	@Override
	public Rotation2d getAndUpdateValue() {
		return new Rotation2d();
	}

	@Override
	public Rotation2d[] asArray() {
		return new Rotation2d[0];
	}

	@Override
	public double getTimestamp() {
		return 0;
	}

	@Override
	public double[] getTimestamps() {
		return new double[0];
	}

	@Override
	public boolean isNear(Rotation2d value, Rotation2d tolerance) {
		return false;
	}

	@Override
	public boolean isFurther(Rotation2d value, Rotation2d tolerance) {
		return false;
	}

	@Override
	public boolean isGreater(Rotation2d value) {
		return false;
	}

	@Override
	public boolean isLess(Rotation2d value) {
		return false;
	}

	@Override
	public void toLog(LogTable logTable) {}

	@Override
	public void fromLog(LogTable logTable) {}

}
