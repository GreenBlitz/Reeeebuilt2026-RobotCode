package frc.robot.hardware.signal;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.TimedValue;
import frc.utils.math.ToleranceMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LogTable;

import java.util.ArrayDeque;

public abstract class DoubleSignal implements InputSignal<Double> {

	private static final String SAMPLES_LOG_SUFFIX = "Samples";
	private static final String TIMESTAMPS_LOG_SUFFIX = "Timestamps";
	private static final Double[] EMPTY_BOXED_SAMPLES = new Double[0];
	private static final double[] EMPTY_PRIMITIVES = new double[0];

	private final String name;
	private final TimedValue<Double> timedValue;

	private boolean isThreaded;
	private ArrayDeque<Double> valueQueue;
	private ArrayDeque<Double> timestampQueue;
	private TimedValue<Double> threadScratchValue;
	private Double[] threadedSamples;
	private double[] threadedPrimitiveSamples;
	private double[] threadedTimestamps;

	public DoubleSignal(String name) {
		this.name = name;
		this.timedValue = new TimedValue<>(0.0, 0);
		this.isThreaded = false;
		this.threadedSamples = EMPTY_BOXED_SAMPLES;
		this.threadedPrimitiveSamples = EMPTY_PRIMITIVES;
		this.threadedTimestamps = EMPTY_PRIMITIVES;
	}

	public void enableThreading(int maxSamples) {
		if (Robot.ROBOT_TYPE.isReal()) {
			updateValue(timedValue);
		}
		this.isThreaded = true;
		this.valueQueue = new ArrayDeque<>(maxSamples);
		this.timestampQueue = new ArrayDeque<>(maxSamples);
		this.threadScratchValue = new TimedValue<>(0.0, 0);
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
			threadedSamples = EMPTY_BOXED_SAMPLES;
			threadedPrimitiveSamples = EMPTY_PRIMITIVES;
			threadedTimestamps = EMPTY_PRIMITIVES;
			valueQueue.clear();
			timestampQueue.clear();
			return;
		}

		threadedSamples = new Double[count];
		threadedPrimitiveSamples = new double[count];
		threadedTimestamps = new double[count];
		for (int i = 0; i < count; i++) {
			double value = valueQueue.pollFirst();
			threadedSamples[i] = value;
			threadedPrimitiveSamples[i] = value;
			threadedTimestamps[i] = timestampQueue.pollFirst();
		}
		valueQueue.clear();
		timestampQueue.clear();

		timedValue.setValue(threadedPrimitiveSamples[count - 1]);
		timedValue.setTimestamp(threadedTimestamps[count - 1]);
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public TimedValue<Double> getLatestTimedValue() {
		return timedValue;
	}

	@Override
	public Double getLatestValue() {
		return timedValue.getValue();
	}

	@Override
	public Double[] asArray() {
		return isThreaded ? threadedSamples : new Double[] {timedValue.getValue()};
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
	public boolean isNear(Double value, Double tolerance) {
		return ToleranceMath.isNear(value, getLatestValue(), tolerance);
	}

	@Override
	public boolean isFurther(Double value, Double tolerance) {
		return !isNear(value, tolerance);
	}

	@Override
	public boolean isGreater(Double value) {
		return getLatestValue() > value;
	}

	@Override
	public boolean isLess(Double value) {
		return getLatestValue() < value;
	}

	@Override
	public void toLog(LogTable table) {
		if (isThreaded) {
			table.put(name, timedValue.getValue());
			table.put(name + SAMPLES_LOG_SUFFIX, threadedPrimitiveSamples);
			table.put(name + TIMESTAMPS_LOG_SUFFIX, threadedTimestamps);
		} else {
			updateValue(timedValue);
			table.put(name, timedValue.getValue());
		}
	}

	@Override
	public void fromLog(LogTable table) {
		timedValue.setValue(table.get(name, 0.0));
		if (isThreaded) {
			threadedPrimitiveSamples = table.get(name + SAMPLES_LOG_SUFFIX, EMPTY_PRIMITIVES);
			threadedSamples = new Double[threadedPrimitiveSamples.length];
			for (int i = 0; i < threadedPrimitiveSamples.length; i++) {
				threadedSamples[i] = threadedPrimitiveSamples[i];
			}
			threadedTimestamps = table.get(name + TIMESTAMPS_LOG_SUFFIX, EMPTY_PRIMITIVES);
			timedValue.setTimestamp(
				threadedTimestamps.length > 0 ? threadedTimestamps[threadedTimestamps.length - 1] : TimeUtil.getCurrentTimeSeconds()
			);
		} else {
			timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
		}
	}

	@Override
	public Double getAndUpdateValue() {
		if (!isThreaded) {
			updateValue(timedValue);
		}
		return timedValue.getValue();
	}

	protected abstract void updateValue(TimedValue<Double> timedValue);

}
