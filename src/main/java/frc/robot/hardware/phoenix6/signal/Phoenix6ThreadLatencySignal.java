package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.phoenix6.Phoenix6OdometryThread;
import frc.robot.hardware.signal.AngleSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LogTable;

import java.util.ArrayDeque;

/**
 * Multi-sample latency-compensated Phoenix 6 {@link AngleSignal} backed by {@link Phoenix6OdometryThread}.
 *
 * <p>Drains all high-frequency samples accumulated since the previous robot cycle into {@link LogTable} during {@code REAL} execution and
 * restores the exact sample and timestamp arrays deterministically during AdvantageKit {@code REPLAY}.
 */
public class Phoenix6ThreadLatencySignal extends AngleSignal implements SignalGetter {

	private static final String SAMPLES_LOG_SUFFIX = "Samples";
	private static final String TIMESTAMPS_LOG_SUFFIX = "Timestamps";
	private static final Rotation2d[] EMPTY_ROTATIONS = new Rotation2d[0];
	private static final double[] EMPTY_TIMESTAMPS = new double[0];

	private final StatusSignal<?> signal;
	private final ArrayDeque<Double> valueQueue;
	private final ArrayDeque<Double> timestampQueue;

	private Rotation2d[] samples;
	private double[] timestamps;

	public Phoenix6ThreadLatencySignal(
		String name,
		StatusSignal<?> signal,
		StatusSignal<?> slopeSignal,
		AngleUnit angleUnit,
		ArrayDeque<Double> valueQueue,
		ArrayDeque<Double> timestampQueue
	) {
		super(name, angleUnit);
		this.signal = signal;
		this.valueQueue = valueQueue;
		this.timestampQueue = timestampQueue;
		this.samples = EMPTY_ROTATIONS;
		this.timestamps = EMPTY_TIMESTAMPS;

		if (Robot.ROBOT_TYPE.isReal() && signal != null && slopeSignal != null) {
			TimedValue<Rotation2d> initialValue = getLatestTimedValue();
			initialValue.setValue(angleUnit.toRotation2d(BaseStatusSignal.getLatencyCompensatedValueAsDouble(signal, slopeSignal)));
			initialValue.setTimestamp(TimeUtil.getRealTimeSeconds());
		}
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
		return timestamps;
	}

	@Override
	protected void updateValue(TimedValue<Rotation2d> timedValue) {
		Phoenix6OdometryThread.ODOMETRY_LOCK.lock();
		try {
			int count = Math.min(valueQueue.size(), timestampQueue.size());
			if (count == 0) {
				samples = EMPTY_ROTATIONS;
				timestamps = EMPTY_TIMESTAMPS;
				valueQueue.clear();
				timestampQueue.clear();
				return;
			}

			samples = new Rotation2d[count];
			timestamps = new double[count];
			for (int i = 0; i < count; i++) {
				samples[i] = angleUnit.toRotation2d(valueQueue.pollFirst());
				timestamps[i] = timestampQueue.pollFirst();
			}
			valueQueue.clear();
			timestampQueue.clear();

			timedValue.setValue(samples[count - 1]);
			timedValue.setTimestamp(timestamps[count - 1]);
		} finally {
			Phoenix6OdometryThread.ODOMETRY_LOCK.unlock();
		}
	}

	@Override
	public void toLog(LogTable table) {
		updateValue(getLatestTimedValue());
		table.put(getName(), getLatestValue());
		table.put(getName() + SAMPLES_LOG_SUFFIX, samples);
		table.put(getName() + TIMESTAMPS_LOG_SUFFIX, timestamps);
	}

	@Override
	public void fromLog(LogTable table) {
		samples = table.get(getName() + SAMPLES_LOG_SUFFIX, EMPTY_ROTATIONS);
		timestamps = table.get(getName() + TIMESTAMPS_LOG_SUFFIX, EMPTY_TIMESTAMPS);
		TimedValue<Rotation2d> latest = getLatestTimedValue();
		latest.setValue(table.get(getName(), latest.getValue()));
		if (timestamps.length > 0) {
			latest.setTimestamp(timestamps[timestamps.length - 1]);
		} else {
			latest.setTimestamp(TimeUtil.getCurrentTimeSeconds());
		}
	}

}
