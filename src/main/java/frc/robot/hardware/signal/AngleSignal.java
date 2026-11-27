package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.math.ToleranceMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LogTable;

import java.util.ArrayDeque;

public abstract class AngleSignal implements InputSignal<Rotation2d> {

	private static final String SAMPLES_LOG_SUFFIX = "Samples";
	private static final String TIMESTAMPS_LOG_SUFFIX = "Timestamps";
	private static final Rotation2d[] EMPTY_SAMPLES = new Rotation2d[0];
	private static final double[] EMPTY_TIMESTAMPS = new double[0];

	private final String name;
	protected final AngleUnit angleUnit;
	private final TimedValue<Rotation2d> timedValue;

	private boolean isThreaded;
	private ArrayDeque<Rotation2d> valueQueue;
	private ArrayDeque<Double> timestampQueue;
	private TimedValue<Rotation2d> threadScratchValue;
	private Rotation2d[] threadedSamples;
	private double[] threadedTimestamps;

	public AngleSignal(String name, AngleUnit angleUnit) {
		this.name = name;
		this.angleUnit = angleUnit;
		this.timedValue = new TimedValue<>(new Rotation2d(), 0);
		this.isThreaded = false;
		this.threadedSamples = EMPTY_SAMPLES;
		this.threadedTimestamps = EMPTY_TIMESTAMPS;
	}

	public void enableThreading(int maxSamples) {
		if (Robot.ROBOT_TYPE.isReal()) {
			updateValue(timedValue);
		}
		this.isThreaded = true;
		this.valueQueue = new ArrayDeque<>(maxSamples);
		this.timestampQueue = new ArrayDeque<>(maxSamples);
		this.threadScratchValue = new TimedValue<>(new Rotation2d(), 0);
	}

	public void recordThreadedSample(double batchTimestampSeconds, int maxSamples) {
		if (!isThreaded || valueQueue == null) {
			return;
		}
		updateValue(threadScratchValue);
		if (valueQueue.size() >= maxSamples) {
			valueQueue.pollFirst();
			timestampQueue.pollFirst();
		}
		valueQueue.offerLast(threadScratchValue.getValue());
		timestampQueue.offerLast(batchTimestampSeconds);
	}

	public void drainThreadedSamples() {
		if (!isThreaded || valueQueue == null) {
			return;
		}
		int count = Math.min(valueQueue.size(), timestampQueue.size());
		if (count == 0) {
			threadedSamples = EMPTY_SAMPLES;
			threadedTimestamps = EMPTY_TIMESTAMPS;
			valueQueue.clear();
			timestampQueue.clear();
			return;
		}

		threadedSamples = new Rotation2d[count];
		threadedTimestamps = new double[count];
		for (int i = 0; i < count; i++) {
			threadedSamples[i] = valueQueue.pollFirst();
			threadedTimestamps[i] = timestampQueue.pollFirst();
		}
		valueQueue.clear();
		timestampQueue.clear();

		timedValue.setValue(threadedSamples[count - 1]);
		timedValue.setTimestamp(threadedTimestamps[count - 1]);
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public TimedValue<Rotation2d> getLatestTimedValue() {
		return timedValue;
	}

	@Override
	public Rotation2d getLatestValue() {
		return timedValue.getValue();
	}

	@Override
	public Rotation2d[] asArray() {
		return isThreaded ? threadedSamples : new Rotation2d[] {timedValue.getValue()};
	}

	@Override
	public double getTimestamp() {
		return timedValue.getTimestamp();
	}

	@Override
	public double[] getTimestamps() {
		return isThreaded ? threadedTimestamps : new double[] {timedValue.getTimestamp()};
	}

	@Override
	public boolean isNear(Rotation2d value, Rotation2d tolerance) {
		return ToleranceMath.isNear(value.getRotations(), getLatestValue().getRotations(), tolerance.getRotations());
	}

	@Override
	public boolean isFurther(Rotation2d value, Rotation2d tolerance) {
		return !isNear(value, tolerance);
	}

	@Override
	public boolean isGreater(Rotation2d value) {
		return getLatestValue().getRotations() > value.getRotations();
	}

	@Override
	public boolean isLess(Rotation2d value) {
		return getLatestValue().getRotations() < value.getRotations();
	}

	@Override
	public void toLog(LogTable table) {
		if (isThreaded) {
			table.put(name, timedValue.getValue());
			table.put(name + SAMPLES_LOG_SUFFIX, threadedSamples);
			table.put(name + TIMESTAMPS_LOG_SUFFIX, threadedTimestamps);
		} else {
			updateValue(timedValue);
			table.put(name, timedValue.getValue());
		}
	}

	@Override
	public void fromLog(LogTable table) {
		timedValue.setValue(table.get(name, new Rotation2d()));
		if (isThreaded) {
			threadedSamples = table.get(name + SAMPLES_LOG_SUFFIX, EMPTY_SAMPLES);
			threadedTimestamps = table.get(name + TIMESTAMPS_LOG_SUFFIX, EMPTY_TIMESTAMPS);
			timedValue.setTimestamp(
				threadedTimestamps.length > 0 ? threadedTimestamps[threadedTimestamps.length - 1] : TimeUtil.getCurrentTimeSeconds()
			);
		} else {
			timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
		}
	}

	@Override
	public Rotation2d getAndUpdateValue() {
		if (!isThreaded) {
			updateValue(timedValue);
		}
		return timedValue.getValue();
	}

	protected abstract void updateValue(TimedValue<Rotation2d> timedValue);

}
