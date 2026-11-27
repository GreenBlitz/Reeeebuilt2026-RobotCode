package frc.robot.hardware.phoenix6.signal.threaded;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.Robot;
import frc.robot.hardware.phoenix6.signal.SignalGetter;
import frc.robot.hardware.signal.DoubleSignal;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LogTable;

import java.util.Arrays;

public class ThreadedDoubleSignal extends DoubleSignal implements SignalGetter {

	private static final String SAMPLES_LOG_SUFFIX = "Samples";
	private static final String TIMESTAMPS_LOG_SUFFIX = "Timestamps";

	private final StatusSignal<?> signal;
	private final SignalSampleQueue sampleQueue;

	private double[] primitiveSamples;
	private Double[] boxedSamples;
	private double[] timestampsSeconds;

	ThreadedDoubleSignal(String name, StatusSignal<?> signal, SignalSampleQueue sampleQueue) {
		super(name);
		this.signal = signal;
		this.sampleQueue = sampleQueue;

		TimedValue<Double> initialValue = getLatestTimedValue();
		if (!Robot.ROBOT_TYPE.isReplay() && signal != null) {
			initialValue.setValue(signal.getValueAsDouble());
			initialValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
		}
		this.primitiveSamples = new double[] {initialValue.getValue()};
		this.boxedSamples = new Double[] {initialValue.getValue()};
		this.timestampsSeconds = new double[] {initialValue.getTimestamp()};
	}

	void recordSample(double timestampSeconds) {
		double value = signal.getValueAsDouble();
		double latencyCompensatedTimestampSeconds = timestampSeconds - signal.getTimestamp().getLatency();
		sampleQueue.insert(value, latencyCompensatedTimestampSeconds);
	}

	@Override
	public StatusSignal<?> getSignal() {
		return signal;
	}

	@Override
	public Double[] asArray() {
		return boxedSamples;
	}

	@Override
	public double[] getTimestamps() {
		return timestampsSeconds;
	}

	@Override
	protected void updateValue(TimedValue<Double> timedValue) {
		if (Robot.ROBOT_TYPE.isReplay()) {
			return;
		}
		sampleQueue.updateCycleSamples();
		double[] rawValues = sampleQueue.getCycleValues();
		double[] rawTimestampsSeconds = sampleQueue.getCycleTimestampsSeconds();
		int sampleCount = Math.min(rawValues.length, rawTimestampsSeconds.length);

		if (sampleCount == 0) {
			double latestValue = timedValue.getValue();
			primitiveSamples = new double[] {latestValue};
			boxedSamples = new Double[] {latestValue};
			timestampsSeconds = new double[] {timedValue.getTimestamp()};
			return;
		}

		primitiveSamples = new double[sampleCount];
		boxedSamples = new Double[sampleCount];
		timestampsSeconds = new double[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			primitiveSamples[i] = rawValues[i];
			boxedSamples[i] = rawValues[i];
			timestampsSeconds[i] = rawTimestampsSeconds[i];
		}
		timedValue.setValue(primitiveSamples[sampleCount - 1]);
		timedValue.setTimestamp(timestampsSeconds[sampleCount - 1]);
	}

	@Override
	public void toLog(LogTable table) {
		updateValue(getLatestTimedValue());
		table.put(getName(), getLatestValue());
		table.put(getName() + SAMPLES_LOG_SUFFIX, primitiveSamples);
		table.put(getName() + TIMESTAMPS_LOG_SUFFIX, timestampsSeconds);
	}

	@Override
	public void fromLog(LogTable table) {
		TimedValue<Double> latest = getLatestTimedValue();
		double fallbackValue = table.get(getName(), latest.getValue());
		primitiveSamples = readSamplesFromLog(table, fallbackValue);
		boxedSamples = toBoxedArray(primitiveSamples);
		timestampsSeconds = readTimestampsFromLog(table, primitiveSamples.length);
		latest.setValue(primitiveSamples[primitiveSamples.length - 1]);
		latest.setTimestamp(timestampsSeconds[timestampsSeconds.length - 1]);
	}

	private double[] readSamplesFromLog(LogTable table, double fallbackValue) {
		double[] loggedSamples = table.get(getName() + SAMPLES_LOG_SUFFIX, new double[] {fallbackValue});
		return loggedSamples.length > 0 ? loggedSamples : new double[] {fallbackValue};
	}

	private Double[] toBoxedArray(double[] values) {
		Double[] boxed = new Double[values.length];
		for (int i = 0; i < values.length; i++) {
			boxed[i] = values[i];
		}
		return boxed;
	}

	private double[] readTimestampsFromLog(LogTable table, int sampleCount) {
		double currentTimeSeconds = TimeUtil.getCurrentTimeSeconds();
		double[] loggedTimestampsSeconds = table.get(getName() + TIMESTAMPS_LOG_SUFFIX, new double[] {currentTimeSeconds});
		if (loggedTimestampsSeconds.length == sampleCount) {
			return loggedTimestampsSeconds;
		}
		double[] fallbackTimestampsSeconds = new double[sampleCount];
		Arrays.fill(fallbackTimestampsSeconds, currentTimeSeconds);
		return fallbackTimestampsSeconds;
	}

}
