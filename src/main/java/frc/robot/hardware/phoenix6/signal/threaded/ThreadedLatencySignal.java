package frc.robot.hardware.phoenix6.signal.threaded;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Robot;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtil;

public class ThreadedLatencySignal extends ThreadedAngleSignal {

	private final StatusSignal<?> slopeSignal;

	ThreadedLatencySignal(String name, StatusSignal<?> signal, StatusSignal<?> slopeSignal, AngleUnit angleUnit, SignalSampleQueue sampleQueue) {
		super(name, signal, angleUnit, sampleQueue);
		this.slopeSignal = slopeSignal;

		if (!Robot.ROBOT_TYPE.isReplay() && signal != null && slopeSignal != null) {
			double compensatedValue = BaseStatusSignal.getLatencyCompensatedValueAsDouble(signal, slopeSignal);
			setInitialSample(angleUnit.toRotation2d(compensatedValue), TimeUtil.getCurrentTimeSeconds());
		}
	}

	@Override
	void recordSample(double timestampSeconds) {
		double compensatedValue = BaseStatusSignal.getLatencyCompensatedValueAsDouble(signal, slopeSignal);
		insertSample(compensatedValue, timestampSeconds);
	}

}
