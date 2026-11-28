package frc.robot.hardware.phoenix6.signal.threaded;

import frc.utils.time.TimeUtil;

class SignalSampleQueue {

	static final int MAX_SAMPLES_PER_CYCLE = 25;
	private static final double[] EMPTY_DOUBLES = new double[0];

	private final Phoenix6SignalThread thread;
	private final double[] bufferedValues;
	private final double[] bufferedTimestampsSeconds;

	private int oldestIndex;
	private int bufferedCount;
	private long droppedSamples;

	private double[] cycleValues;
	private double[] cycleTimestampsSeconds;

	SignalSampleQueue(Phoenix6SignalThread thread) {
		this.thread = thread;
		this.bufferedValues = new double[MAX_SAMPLES_PER_CYCLE];
		this.bufferedTimestampsSeconds = new double[MAX_SAMPLES_PER_CYCLE];
		this.oldestIndex = 0;
		this.bufferedCount = 0;
		this.droppedSamples = 0;
		this.cycleValues = EMPTY_DOUBLES;
		this.cycleTimestampsSeconds = EMPTY_DOUBLES;
	}

	void insert(double value, double timestampSeconds) {
		int writeIndex = (oldestIndex + bufferedCount) % bufferedValues.length;
		bufferedValues[writeIndex] = value;
		bufferedTimestampsSeconds[writeIndex] = timestampSeconds;
		if (bufferedCount < bufferedValues.length) {
			bufferedCount++;
		} else {
			oldestIndex = (oldestIndex + 1) % bufferedValues.length;
			droppedSamples++;
		}
	}

	void drainBufferedSamples() {
		if (bufferedCount == 0) {
			cycleValues = EMPTY_DOUBLES;
			cycleTimestampsSeconds = EMPTY_DOUBLES;
			return;
		}

		double[] values = new double[bufferedCount];
		double[] timestampsSeconds = new double[bufferedCount];
		for (int i = 0; i < bufferedCount; i++) {
			int bufferIndex = (oldestIndex + i) % bufferedValues.length;
			values[i] = bufferedValues[bufferIndex];
			timestampsSeconds[i] = bufferedTimestampsSeconds[bufferIndex];
		}
		oldestIndex = 0;
		bufferedCount = 0;
		cycleValues = values;
		cycleTimestampsSeconds = timestampsSeconds;
	}

	void updateCycleSamples() {
		if (thread != null) {
			double cycleStartTimeSeconds = TimeUtil.getCurrentCycleStartingTimeSeconds() > 0
				? TimeUtil.getCurrentCycleStartingTimeSeconds()
				: TimeUtil.getCurrentTimeSeconds();
			thread.updateCycleSamples(cycleStartTimeSeconds);
		}
	}

	double[] getCycleValues() {
		return cycleValues;
	}

	double[] getCycleTimestampsSeconds() {
		return cycleTimestampsSeconds;
	}

	long getDroppedSamples() {
		return droppedSamples;
	}

}
