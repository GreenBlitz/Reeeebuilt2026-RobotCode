package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.signal.SignalGetter;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Per-{@link BusChain} high-frequency Phoenix 6 signal sampling thread.
 *
 * <p>Owned internally by {@link BusChain}: blocks on {@link BaseStatusSignal#waitForAll} on CAN FD networks (or timed {@code refreshAll} on
 * non-FD networks), pushes synchronized samples into each registered signal's queue, and drains all queues in atomic lockstep inside
 * {@link BusChain#refreshAll()}.
 */
class Phoenix6SignalThread extends Thread {

	static final int MAX_SAMPLES_PER_CYCLE = 25;

	private static final int THREAD_REAL_TIME_PRIORITY = 20;
	private static final double FREQUENCY_WINDOW_SECONDS = 1.0;

	private final ReentrantLock lock;
	private final CANBus canBus;
	private final List<BaseStatusSignal> statusSignals;
	private final List<SignalGetter> signalWrappers;

	private BaseStatusSignal[] signalsArray;
	private boolean isCANFD;
	private double targetFrequencyHertz;
	private volatile boolean isRunning;
	private boolean alertRegistered;

	private int validUpdatesSinceLastLog;
	private int validUpdatesInFrequencyWindow;
	private long totalFailedUpdates;
	private double frequencyWindowStartSeconds;
	private double effectiveFrequencyHertz;

	Phoenix6SignalThread(String busName, CANBus canBus) {
		super("Phoenix6SignalThread-" + busName);
		setDaemon(true);
		this.lock = new ReentrantLock();
		this.canBus = canBus;
		this.statusSignals = new ArrayList<>();
		this.signalWrappers = new ArrayList<>();
		this.signalsArray = new BaseStatusSignal[0];
		this.isCANFD = false;
		this.targetFrequencyHertz = RobotConstants.ODOMETRY_SIGNALS_FREQUENCY_HERTZ;
		this.isRunning = false;
		this.alertRegistered = false;
	}

	void registerSignal(SignalGetter signalWrapper, double frequencyHertz, BaseStatusSignal... rawSignals) {
		signalWrapper.enableThreading(MAX_SAMPLES_PER_CYCLE);
		if (!Robot.ROBOT_TYPE.isReal()) {
			return;
		}

		lock.lock();
		try {
			targetFrequencyHertz = Math.max(targetFrequencyHertz, frequencyHertz);
			signalWrappers.add(signalWrapper);
			for (BaseStatusSignal rawSignal : rawSignals) {
				addUniqueStatusSignal(rawSignal);
			}
		} finally {
			lock.unlock();
		}
	}

	private void addUniqueStatusSignal(BaseStatusSignal candidate) {
		if (candidate == null) {
			return;
		}
		for (BaseStatusSignal existing : statusSignals) {
			if (existing == candidate) {
				return;
			}
		}
		statusSignals.add(candidate);
	}

	boolean hasRegisteredSignals() {
		return !signalWrappers.isEmpty();
	}

	void refreshLatestSamples() {
		if (signalWrappers.isEmpty()) {
			return;
		}
		startIfNeeded();

		lock.lock();
		try {
			for (SignalGetter wrapper : signalWrappers) {
				wrapper.drainThreadedSamples();
			}
		} finally {
			lock.unlock();
		}
	}

	private synchronized void startIfNeeded() {
		if (isRunning || !RobotConstants.ENABLE_ODOMETRY_THREAD || !Robot.ROBOT_TYPE.isReal() || statusSignals.isEmpty()) {
			return;
		}
		lock.lock();
		try {
			isCANFD = canBus.isNetworkFD();
			signalsArray = statusSignals.toArray(BaseStatusSignal[]::new);
			frequencyWindowStartSeconds = TimeUtil.getRealTimeSeconds();
		} finally {
			lock.unlock();
		}

		isRunning = true;
		super.start();
	}

	@Override
	public void run() {
		if (isCANFD) {
			BaseStatusSignal.setUpdateFrequencyForAll(targetFrequencyHertz, signalsArray);
		}
		Threads.setCurrentThreadPriority(true, THREAD_REAL_TIME_PRIORITY);
		double waitTimeoutSeconds = 2.0 / targetFrequencyHertz;
		long sleepTimeMillis = Math.max(1L, Math.round(1000.0 / targetFrequencyHertz));

		while (isRunning) {
			StatusCode statusCode;
			if (isCANFD) {
				statusCode = BaseStatusSignal.waitForAll(waitTimeoutSeconds, signalsArray);
			} else {
				try {
					Thread.sleep(sleepTimeMillis);
				} catch (InterruptedException interruptedException) {
					Thread.currentThread().interrupt();
					break;
				}
				statusCode = BaseStatusSignal.refreshAll(signalsArray);
			}

			double batchTimestampSeconds = TimeUtil.getRealTimeSeconds();
			recordBatch(statusCode, batchTimestampSeconds);
		}
	}

	private void recordBatch(StatusCode batchStatus, double batchTimestampSeconds) {
		boolean anySignalValid = false;
		for (BaseStatusSignal signal : signalsArray) {
			if (signal.getStatus().isOK()) {
				anySignalValid = true;
				break;
			}
		}

		lock.lock();
		try {
			if (!anySignalValid) {
				totalFailedUpdates++;
				return;
			}
			if (!batchStatus.isOK()) {
				totalFailedUpdates++;
			}

			for (SignalGetter wrapper : signalWrappers) {
				wrapper.recordThreadedSample(batchTimestampSeconds, MAX_SAMPLES_PER_CYCLE);
			}

			validUpdatesSinceLastLog++;
			validUpdatesInFrequencyWindow++;
		} finally {
			lock.unlock();
		}
	}

	void logStatus(String logPath) {
		if (signalWrappers.isEmpty()) {
			return;
		}
		if (!alertRegistered && RobotConstants.ENABLE_ODOMETRY_THREAD && Robot.ROBOT_TYPE.isReal()) {
			alertRegistered = true;
			AlertManager
				.addAlert(new PeriodicAlert(Alert.AlertType.WARNING, logPath + "/StalledAt", () -> isRunning && validUpdatesSinceLastLog == 0));
		}

		int updatesThisCycle;
		long failedTotal;
		double measuredFrequencyHz;

		lock.lock();
		try {
			updatesThisCycle = validUpdatesSinceLastLog;
			validUpdatesSinceLastLog = 0;
			failedTotal = totalFailedUpdates;

			double nowSeconds = TimeUtil.getRealTimeSeconds();
			double windowElapsedSeconds = nowSeconds - frequencyWindowStartSeconds;
			if (windowElapsedSeconds >= FREQUENCY_WINDOW_SECONDS) {
				effectiveFrequencyHertz = validUpdatesInFrequencyWindow / windowElapsedSeconds;
				validUpdatesInFrequencyWindow = 0;
				frequencyWindowStartSeconds = nowSeconds;
			}
			measuredFrequencyHz = effectiveFrequencyHertz;
		} finally {
			lock.unlock();
		}

		Logger.recordOutput(logPath + "/Running", isRunning);
		Logger.recordOutput(logPath + "/IsCANFD", isCANFD);
		Logger.recordOutput(logPath + "/RegisteredSignalsCount", signalsArray.length);
		Logger.recordOutput(logPath + "/ValidUpdatesLastCycle", updatesThisCycle);
		Logger.recordOutput(logPath + "/FailedUpdatesTotal", failedTotal);
		Logger.recordOutput(logPath + "/EffectiveFrequencyHz", measuredFrequencyHz);
	}

}
