package frc.robot.hardware.phoenix6.signal.threaded;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

public class ThreadedLatencySignal extends ThreadedAngleSignal {

	private final StatusSignal<?> slopeSignal;

	ThreadedLatencySignal(String name, StatusSignal<?> signal, StatusSignal<?> slopeSignal, AngleUnit angleUnit, SignalSampleQueue sampleQueue) {
		super(name, signal, angleUnit, sampleQueue);
		this.slopeSignal = slopeSignal;

		if (!Robot.ROBOT_TYPE.isReplay() && signal != null && slopeSignal != null) {
			TimedValue<Rotation2d> initialValue = getLatestTimedValue();
			double compensatedValue = BaseStatusSignal.getLatencyCompensatedValueAsDouble(signal, slopeSignal);
			initialValue.setValue(angleUnit.toRotation2d(compensatedValue));
			initialValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
			updateValue(initialValue);
		}
	}

	@Override
	void recordSample(double timestampSeconds) {
		double compensatedValue = BaseStatusSignal.getLatencyCompensatedValueAsDouble(signal, slopeSignal);
		insertSample(compensatedValue, timestampSeconds);
	}

}
