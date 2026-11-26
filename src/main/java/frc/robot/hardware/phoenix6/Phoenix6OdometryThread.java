package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Asynchronous high-frequency odometry sampling thread for Phoenix 6 devices.
 *
 * <p>On a CANivore (CAN FD) bus with Phoenix Pro, {@link BaseStatusSignal#waitForAll} blocks in kernel space until a synchronized batch of
 * status signals arrives (default 250 Hz / 4 ms period), latency-compensates each signal to the live hardware FPGA timestamp, and pushes a
 * lockstep sample across all registered queues under {@link #ODOMETRY_LOCK}.
 */
public class Phoenix6OdometryThread extends Thread {

	public static final ReentrantLock ODOMETRY_LOCK = new ReentrantLock();
	public static final int MAX_SAMPLES_PER_CYCLE = 25;

	private static final String THREAD_NAME = "Phoenix6OdometryThread";
	private static final int THREAD_REAL_TIME_PRIORITY = 20;
	private static final int UPDATE_FREQUENCY_RETRIES = 5;
	private static final double FREQUENCY_WINDOW_SECONDS = 1.0;

	public record SignalQueues(ArrayDeque<Double> values, ArrayDeque<Double> timestamps, StatusSignal<?> clonedSignal) {}

	private record RegisteredEntry(
		StatusSignal<?> signal,
		StatusSignal<?> slopeSignal,
		ArrayDeque<Double> values,
		ArrayDeque<Double> timestamps
	) {}

	private static Phoenix6OdometryThread instance;

	private final List<BaseStatusSignal> registeredStatusSignals;
	private final List<RegisteredEntry> registeredEntries;

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

	private Phoenix6OdometryThread() {
		super(THREAD_NAME);
		setDaemon(true);
		this.registeredStatusSignals = new ArrayList<>();
		this.registeredEntries = new ArrayList<>();
		this.signalsArray = new BaseStatusSignal[0];
		this.isCANFD = true;
		this.targetFrequencyHertz = RobotConstants.ODOMETRY_SIGNALS_FREQUENCY_HERTZ;
		this.isRunning = false;
		this.alertRegistered = false;
		this.validUpdatesSinceLastLog = 0;
		this.validUpdatesInFrequencyWindow = 0;
		this.totalFailedUpdates = 0;
		this.frequencyWindowStartSeconds = 0;
		this.effectiveFrequencyHertz = 0;
	}

	public static synchronized Phoenix6OdometryThread getInstance() {
		if (instance == null) {
			instance = new Phoenix6OdometryThread();
		}
		return instance;
	}

	public SignalQueues registerLatencySignal(StatusSignal<?> signal, StatusSignal<?> slopeSignal, double frequencyHertz, BusChain busChain) {
		ArrayDeque<Double> values = new ArrayDeque<>(MAX_SAMPLES_PER_CYCLE);
		ArrayDeque<Double> timestamps = new ArrayDeque<>(MAX_SAMPLES_PER_CYCLE);

		if (!Robot.ROBOT_TYPE.isReal()) {
			return new SignalQueues(values, timestamps, signal);
		}

		StatusSignal<?> signalClone = signal.clone();
		StatusSignal<?> slopeClone = slopeSignal.clone();
		Phoenix6Util.checkStatusCodeWithRetry(() -> signalClone.setUpdateFrequency(frequencyHertz), UPDATE_FREQUENCY_RETRIES);
		Phoenix6Util.checkStatusCodeWithRetry(() -> slopeClone.setUpdateFrequency(frequencyHertz), UPDATE_FREQUENCY_RETRIES);
		signalClone.refresh();
		slopeClone.refresh();

		ODOMETRY_LOCK.lock();
		try {
			boolean busIsFD = busChain.getCANBus().isNetworkFD();
			isCANFD = registeredEntries.isEmpty() ? busIsFD : (isCANFD && busIsFD);
			targetFrequencyHertz = frequencyHertz;
			registeredStatusSignals.add(signalClone);
			registeredStatusSignals.add(slopeClone);
			registeredEntries.add(new RegisteredEntry(signalClone, slopeClone, values, timestamps));
		} finally {
			ODOMETRY_LOCK.unlock();
		}

		return new SignalQueues(values, timestamps, signalClone);
	}

	@Override
	public synchronized void start() {
		if (isRunning || !RobotConstants.ENABLE_ODOMETRY_THREAD || !Robot.ROBOT_TYPE.isReal()) {
			return;
		}
		ODOMETRY_LOCK.lock();
		try {
			if (registeredEntries.isEmpty()) {
				return;
			}
			signalsArray = registeredStatusSignals.toArray(BaseStatusSignal[]::new);
			frequencyWindowStartSeconds = TimeUtil.getRealTimeSeconds();
		} finally {
			ODOMETRY_LOCK.unlock();
		}

		isRunning = true;
		super.start();
	}

	@Override
	public void run() {
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
				if (statusCode == StatusCode.InvalidNetwork) {
					statusCode = refreshSignalsIndividually();
				}
			}

			double timestampSeconds = TimeUtil.getRealTimeSeconds();
			recordSample(statusCode, timestampSeconds);
		}
	}

	private StatusCode refreshSignalsIndividually() {
		StatusCode overallStatus = StatusCode.OK;
		for (BaseStatusSignal signal : signalsArray) {
			StatusCode status = BaseStatusSignal.refreshAll(signal);
			if (!status.isOK()) {
				overallStatus = status;
			}
		}
		return overallStatus;
	}

	private void recordSample(StatusCode batchStatus, double timestampSeconds) {
		boolean anySignalValid = false;
		for (BaseStatusSignal signal : signalsArray) {
			if (signal.getStatus().isOK()) {
				anySignalValid = true;
				break;
			}
		}

		ODOMETRY_LOCK.lock();
		try {
			if (!anySignalValid) {
				totalFailedUpdates++;
				return;
			}
			if (!batchStatus.isOK()) {
				totalFailedUpdates++;
			}

			for (RegisteredEntry entry : registeredEntries) {
				double value = computeSampleValue(entry);
				offerWithRingEviction(entry.values(), value);
				offerWithRingEviction(entry.timestamps(), timestampSeconds);
			}

			validUpdatesSinceLastLog++;
			validUpdatesInFrequencyWindow++;
		} finally {
			ODOMETRY_LOCK.unlock();
		}
	}

	private static double computeSampleValue(RegisteredEntry entry) {
		if (entry.slopeSignal() != null && entry.signal().getStatus().isOK() && entry.slopeSignal().getStatus().isOK()) {
			return BaseStatusSignal.getLatencyCompensatedValueAsDouble(entry.signal(), entry.slopeSignal());
		}
		return entry.signal().getValueAsDouble();
	}

	static void offerWithRingEviction(ArrayDeque<Double> queue, double value) {
		if (queue.size() >= MAX_SAMPLES_PER_CYCLE) {
			queue.pollFirst();
		}
		queue.offerLast(value);
	}

	public void logStatus(String logPath) {
		if (!alertRegistered && RobotConstants.ENABLE_ODOMETRY_THREAD && Robot.ROBOT_TYPE.isReal()) {
			alertRegistered = true;
			AlertManager
				.addAlert(new PeriodicAlert(Alert.AlertType.WARNING, logPath + "/StalledAt", () -> isRunning && validUpdatesSinceLastLog == 0));
		}

		int updatesThisCycle;
		long failedTotal;
		double measuredFrequencyHz;

		ODOMETRY_LOCK.lock();
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
			ODOMETRY_LOCK.unlock();
		}

		Logger.recordOutput(logPath + "/Running", isRunning);
		Logger.recordOutput(logPath + "/IsCANFD", isCANFD);
		Logger.recordOutput(logPath + "/ValidUpdatesLastCycle", updatesThisCycle);
		Logger.recordOutput(logPath + "/FailedUpdatesTotal", failedTotal);
		Logger.recordOutput(logPath + "/EffectiveFrequencyHz", measuredFrequencyHz);
	}

}
