package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;

public interface SignalGetter {

	/**
	 * For using refresh all with more signals...
	 */
	StatusSignal<?> getSignal();

	void enableThreading(int maxSamples);

	void recordThreadedSample(double batchTimestampSeconds, int maxSamples);

	void drainThreadedSamples();

}
