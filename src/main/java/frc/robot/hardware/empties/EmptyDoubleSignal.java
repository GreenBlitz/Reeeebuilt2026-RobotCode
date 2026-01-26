package frc.robot.hardware.empties;

import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.TimedValue;
import org.littletonrobotics.junction.LogTable;

public class EmptyDoubleSignal implements InputSignal<Double> {

	@Override
	public String getName() {
		return "";
	}

	@Override
	public TimedValue<Double> getLatestTimedValue() {
		return new TimedValue<>(0.0, 0);
	}

	@Override
	public Double getLatestValue() {
		return 0.0;
	}

	@Override
	public Double getAndUpdateValue() {
		return 0.0;
	}

	@Override
	public Double[] asArray() {
		return new Double[0];
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
	public boolean isNear(Double value, Double tolerance) {
		return false;
	}

	@Override
	public boolean isFurther(Double value, Double tolerance) {
		return false;
	}

	@Override
	public boolean isGreater(Double value) {
		return false;
	}

	@Override
	public boolean isLess(Double value) {
		return false;
	}

	@Override
	public void toLog(LogTable logTable) {}

	@Override
	public void fromLog(LogTable logTable) {}

}
