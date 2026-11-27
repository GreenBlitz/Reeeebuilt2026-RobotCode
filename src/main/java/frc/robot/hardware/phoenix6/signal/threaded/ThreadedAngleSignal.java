package frc.robot.hardware.phoenix6.signal.threaded;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.phoenix6.signal.SignalGetter;
import frc.robot.hardware.signal.AngleSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LogTable;

import java.util.Arrays;

public class ThreadedAngleSignal extends AngleSignal implements SignalGetter {

	private static final String SAMPLES_LOG_SUFFIX = "Samples";
	private static final String TIMESTAMPS_LOG_SUFFIX = "Timestamps";

	protected final StatusSignal<?> signal;
	private final SignalSampleQueue sampleQueue;

	private Rotation2d[] samples;
	private double[] timestampsSeconds;

	ThreadedAngleSignal(String name, StatusSignal<?> signal, AngleUnit angleUnit, SignalSampleQueue sampleQueue) {
		super(name, angleUnit);
		this.signal = signal;
		this.sampleQueue = sampleQueue;

		TimedValue<Rotation2d> initialValue = getLatestTimedValue();
		if (!Robot.ROBOT_TYPE.isReplay() && signal != null) {
			initialValue.setValue(angleUnit.toRotation2d(signal.getValueAsDouble()));
			initialValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
		}
		this.samples = new Rotation2d[] {initialValue.getValue()};
		this.timestampsSeconds = new double[] {initialValue.getTimestamp()};
	}

	protected void insertSample(double rawValue, double timestampSeconds) {
		sampleQueue.insert(rawValue, timestampSeconds);
	}

	void recordSample(double timestampSeconds) {
		double rawValue = signal.getValueAsDouble();
		double latencyCompensatedTimestampSeconds = timestampSeconds - signal.getTimestamp().getLatency();
		insertSample(rawValue, latencyCompensatedTimestampSeconds);
	}

	@Override
	public StatusSignal<?> getSignal() {
		return signal;
	}

	@Override
	public Rotation2d[] asArray() {
		return samples;
	}

	@Override
	public double[] getTimestamps() {
		return timestampsSeconds;
	}

	@Override
	protected void updateValue(TimedValue<Rotation2d> timedValue) {
		if (Robot.ROBOT_TYPE.isReplay()) {
			return;
		}
		sampleQueue.updateCycleSamples();
		double[] rawValues = sampleQueue.getCycleValues();
		double[] rawTimestampsSeconds = sampleQueue.getCycleTimestampsSeconds();
		int sampleCount = Math.min(rawValues.length, rawTimestampsSeconds.length);

		if (sampleCount == 0) {
			samples = new Rotation2d[] {timedValue.getValue()};
			timestampsSeconds = new double[] {timedValue.getTimestamp()};
			return;
		}

		samples = new Rotation2d[sampleCount];
		timestampsSeconds = new double[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			samples[i] = angleUnit.toRotation2d(rawValues[i]);
			timestampsSeconds[i] = rawTimestampsSeconds[i];
		}
		timedValue.setValue(samples[sampleCount - 1]);
		timedValue.setTimestamp(timestampsSeconds[sampleCount - 1]);
	}

	@Override
	public void toLog(LogTable table) {
		updateValue(getLatestTimedValue());
		table.put(getName(), getLatestValue());
		table.put(getName() + SAMPLES_LOG_SUFFIX, samples);
		table.put(getName() + TIMESTAMPS_LOG_SUFFIX, timestampsSeconds);
	}

	@Override
	public void fromLog(LogTable table) {
		TimedValue<Rotation2d> latest = getLatestTimedValue();
		Rotation2d fallbackValue = table.get(getName(), latest.getValue());
		samples = readSamplesFromLog(table, fallbackValue);
		timestampsSeconds = readTimestampsFromLog(table, samples.length);
		latest.setValue(samples[samples.length - 1]);
		latest.setTimestamp(timestampsSeconds[timestampsSeconds.length - 1]);
	}

	private Rotation2d[] readSamplesFromLog(LogTable table, Rotation2d fallbackValue) {
		Rotation2d[] loggedSamples = table.get(getName() + SAMPLES_LOG_SUFFIX, new Rotation2d[] {fallbackValue});
		return loggedSamples.length > 0 ? loggedSamples : new Rotation2d[] {fallbackValue};
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
