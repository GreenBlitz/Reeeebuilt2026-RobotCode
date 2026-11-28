package frc.robot.hardware.phoenix6.signal.threaded;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Timestamp;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.Robot;
import frc.robot.hardware.phoenix6.BusChain;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleConsumer;

class Phoenix6SignalThread extends Thread {

	private static final int THREAD_REAL_TIME_PRIORITY = 20;
	private static final double DEFAULT_FREQUENCY_HERTZ = 250.0;
	private static final double FREQUENCY_WINDOW_SECONDS = 1.0;
	private static final double MAX_SIGNAL_STALE_PERIODS = 5.0;
	private static final int STALLED_CYCLE_THRESHOLD = 5;

	private static final Map<BusChain, Phoenix6SignalThread> THREADS = new EnumMap<>(BusChain.class);

	static synchronized Phoenix6SignalThread getOrCreate(BusChain busChain) {
		return THREADS.computeIfAbsent(busChain, Phoenix6SignalThread::new);
	}

	private record RegisteredSignal(SignalSampleQueue queue, DoubleConsumer sampleRecorder) {}

	private record ActiveBatch(BaseStatusSignal[] statusSignals, RegisteredSignal[] registeredSignals) {}

	private final ReentrantLock lock;
	private final CANBus canBus;
	private final String logPath;
	private final List<BaseStatusSignal> statusSignals;
	private final List<RegisteredSignal> registeredSignals;

	private volatile ActiveBatch activeBatch;
	private volatile boolean hasNewSignals;
	private volatile boolean isRunning;
	private volatile double targetFrequencyHertz;
	private volatile double minAppliedFrequencyHertz;
	private boolean isCANFD;
	private double[] lastRecordedSignalTimestampsSeconds;
	private double lastCycleStartTimeSeconds;
	private boolean isAlertRegistered;

	private int validUpdatesSinceLastCycle;
	private int validUpdatesInWindow;
	private int consecutiveEmptyCycles;
	private long totalFailedUpdates;
	private double windowStartTimeSeconds;
	private double effectiveFrequencyHertz;

	Phoenix6SignalThread(BusChain busChain) {
		this(busChain.getCANBus().getName(), busChain.getCANBus());
	}

	Phoenix6SignalThread(String busName, CANBus canBus) {
		super("Phoenix6SignalThread-" + busName);
		setDaemon(true);
		this.lock = new ReentrantLock();
		this.canBus = canBus;
		this.logPath = "Bus/" + busName + "/SignalThread";
		this.statusSignals = new ArrayList<>();
		this.registeredSignals = new ArrayList<>();
		this.activeBatch = new ActiveBatch(new BaseStatusSignal[0], new RegisteredSignal[0]);
		this.hasNewSignals = false;
		this.isRunning = false;
		this.targetFrequencyHertz = DEFAULT_FREQUENCY_HERTZ;
		this.minAppliedFrequencyHertz = 0.0;
		this.isCANFD = false;
		this.lastRecordedSignalTimestampsSeconds = new double[0];
		this.lastCycleStartTimeSeconds = Double.NaN;
		this.isAlertRegistered = false;
	}

	SignalSampleQueue createQueue() {
		return new SignalSampleQueue(this);
	}

	void registerSignal(SignalSampleQueue queue, DoubleConsumer sampleRecorder, double frequencyHertz, BaseStatusSignal... signals) {
		lock.lock();
		try {
			targetFrequencyHertz = Math.max(targetFrequencyHertz, frequencyHertz);
			registeredSignals.add(new RegisteredSignal(queue, sampleRecorder));
			if (!Robot.ROBOT_TYPE.isReplay()) {
				for (BaseStatusSignal signal : signals) {
					addUniqueSignal(signal);
				}
			}
			hasNewSignals = true;
		} finally {
			lock.unlock();
		}
	}

	private void addUniqueSignal(BaseStatusSignal candidate) {
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

	void updateCycleSamples(double cycleStartTimeSeconds) {
		if (Double.compare(cycleStartTimeSeconds, lastCycleStartTimeSeconds) == 0) {
			return;
		}
		lastCycleStartTimeSeconds = cycleStartTimeSeconds;

		boolean wasRunningBeforeCycle = isRunning;
		boolean hasPeriodicStarted = TimeUtil.getCurrentCycleStartingTimeSeconds() > 0;
		if (hasPeriodicStarted) {
			startIfReady();
		}

		int updatesLastCycle;
		long failedUpdates;
		long droppedBatches;
		double measuredFrequencyHertz;

		lock.lock();
		try {
			droppedBatches = 0;
			for (RegisteredSignal registeredSignal : registeredSignals) {
				registeredSignal.queue().drainBufferedSamples();
				droppedBatches = Math.max(droppedBatches, registeredSignal.queue().getDroppedSamples());
			}

			updatesLastCycle = validUpdatesSinceLastCycle;
			validUpdatesSinceLastCycle = 0;
			failedUpdates = totalFailedUpdates;

			if (wasRunningBeforeCycle) {
				consecutiveEmptyCycles = updatesLastCycle == 0 ? consecutiveEmptyCycles + 1 : 0;
			}

			updateFrequencyWindow(TimeUtil.getRealTimeSeconds());
			measuredFrequencyHertz = effectiveFrequencyHertz;
		} finally {
			lock.unlock();
		}

		if (hasPeriodicStarted && !Robot.ROBOT_TYPE.isReplay()) {
			logStatus(updatesLastCycle, failedUpdates, droppedBatches, measuredFrequencyHertz);
		}
	}

	private void updateFrequencyWindow(double currentTimeSeconds) {
		double elapsedSeconds = currentTimeSeconds - windowStartTimeSeconds;
		if (elapsedSeconds >= FREQUENCY_WINDOW_SECONDS) {
			effectiveFrequencyHertz = validUpdatesInWindow / elapsedSeconds;
			validUpdatesInWindow = 0;
			windowStartTimeSeconds = currentTimeSeconds;
		}
	}

	private void logStatus(int updatesLastCycle, long failedUpdates, long droppedBatches, double measuredFrequencyHertz) {
		if (!isAlertRegistered) {
			isAlertRegistered = true;
			AlertManager.addAlert(
				new PeriodicAlert(
					Alert.AlertType.WARNING,
					logPath + "/StalledAt",
					() -> isRunning && consecutiveEmptyCycles >= STALLED_CYCLE_THRESHOLD
				)
			);
		}

		Logger.recordOutput(logPath + "/Running", isRunning);
		Logger.recordOutput(logPath + "/IsCANFD", isCANFD);
		Logger.recordOutput(logPath + "/RegisteredSignalsCount", activeBatch.statusSignals().length);
		Logger.recordOutput(logPath + "/ValidUpdatesLastCycle", updatesLastCycle);
		Logger.recordOutput(logPath + "/FailedUpdatesTotal", failedUpdates);
		Logger.recordOutput(logPath + "/DroppedBatchesTotal", droppedBatches);
		Logger.recordOutput(logPath + "/MinAppliedFrequencyHz", minAppliedFrequencyHertz);
		Logger.recordOutput(logPath + "/EffectiveFrequencyHz", measuredFrequencyHertz);
	}

	private synchronized void startIfReady() {
		if (isRunning || Robot.ROBOT_TYPE.isReplay() || canBus == null) {
			return;
		}
		lock.lock();
		try {
			if (statusSignals.isEmpty()) {
				return;
			}
			isCANFD = canBus.isNetworkFD();
			activeBatch = new ActiveBatch(statusSignals.toArray(BaseStatusSignal[]::new), registeredSignals.toArray(RegisteredSignal[]::new));
			hasNewSignals = true;
			windowStartTimeSeconds = TimeUtil.getRealTimeSeconds();
		} finally {
			lock.unlock();
		}

		isRunning = true;
		super.start();
	}

	@Override
	public void run() {
		Threads.setCurrentThreadPriority(true, THREAD_REAL_TIME_PRIORITY);
		while (isRunning) {
			ActiveBatch currentBatch = getActiveBatch();
			StatusCode statusCode = waitForNextBatch(currentBatch.statusSignals());
			if (Thread.currentThread().isInterrupted()) {
				break;
			}
			recordValidSamples(currentBatch, statusCode);
		}
	}

	private StatusCode waitForNextBatch(BaseStatusSignal[] activeSignals) {
		if (isCANFD) {
			double waitTimeoutSeconds = 2.0 / targetFrequencyHertz;
			return BaseStatusSignal.waitForAll(waitTimeoutSeconds, activeSignals);
		}

		long sleepMillis = Math.max(1L, Math.round(1000.0 / targetFrequencyHertz));
		try {
			Thread.sleep(sleepMillis);
		} catch (InterruptedException interruptedException) {
			Thread.currentThread().interrupt();
			return StatusCode.GeneralError;
		}
		return BaseStatusSignal.refreshAll(activeSignals);
	}

	private ActiveBatch getActiveBatch() {
		ActiveBatch currentBatch = activeBatch;
		if (!hasNewSignals) {
			return currentBatch;
		}

		lock.lock();
		try {
			if (hasNewSignals) {
				currentBatch = new ActiveBatch(
					statusSignals.toArray(BaseStatusSignal[]::new),
					registeredSignals.toArray(RegisteredSignal[]::new)
				);
				activeBatch = currentBatch;
				hasNewSignals = false;
			}
		} finally {
			lock.unlock();
		}

		BaseStatusSignal[] currentSignals = currentBatch.statusSignals();
		if (isCANFD && currentSignals.length > 0) {
			BaseStatusSignal.setUpdateFrequencyForAll(targetFrequencyHertz, currentSignals);
		}
		minAppliedFrequencyHertz = computeMinAppliedFrequencyHertz(currentSignals);
		return currentBatch;
	}

	private double computeMinAppliedFrequencyHertz(BaseStatusSignal[] signals) {
		if (signals.length == 0) {
			return 0.0;
		}
		double minFrequencyHertz = Double.POSITIVE_INFINITY;
		for (BaseStatusSignal signal : signals) {
			minFrequencyHertz = Math.min(minFrequencyHertz, signal.getAppliedUpdateFrequency());
		}
		return minFrequencyHertz;
	}

	private boolean isBatchValid(BaseStatusSignal[] activeSignals, StatusCode batchStatus) {
		if (activeSignals.length == 0 || !batchStatus.isOK() || !BaseStatusSignal.isAllGood(activeSignals)) {
			return false;
		}
		if (isCANFD) {
			return true;
		}
		return haveAllSignalsUpdated(activeSignals) && areAllSignalsWithinMaxAge(activeSignals);
	}

	private boolean haveAllSignalsUpdated(BaseStatusSignal[] activeSignals) {
		syncLastRecordedTimestampsLength(activeSignals.length);
		for (int i = 0; i < activeSignals.length; i++) {
			Timestamp systemTimestamp = activeSignals[i].getAllTimestamps().getSystemTimestamp();
			if (!systemTimestamp.isValid() || Double.compare(systemTimestamp.getTime(), lastRecordedSignalTimestampsSeconds[i]) == 0) {
				return false;
			}
		}
		return true;
	}

	private void updateLastRecordedSignalTimestamps(BaseStatusSignal[] activeSignals) {
		syncLastRecordedTimestampsLength(activeSignals.length);
		for (int i = 0; i < activeSignals.length; i++) {
			lastRecordedSignalTimestampsSeconds[i] = activeSignals[i].getAllTimestamps().getSystemTimestamp().getTime();
		}
	}

	private void syncLastRecordedTimestampsLength(int signalCount) {
		if (lastRecordedSignalTimestampsSeconds.length == signalCount) {
			return;
		}
		int previousLength = lastRecordedSignalTimestampsSeconds.length;
		lastRecordedSignalTimestampsSeconds = Arrays.copyOf(lastRecordedSignalTimestampsSeconds, signalCount);
		if (signalCount > previousLength) {
			Arrays.fill(lastRecordedSignalTimestampsSeconds, previousLength, signalCount, Double.NaN);
		}
	}

	private boolean areAllSignalsWithinMaxAge(BaseStatusSignal[] activeSignals) {
		double maxAgeSeconds = MAX_SIGNAL_STALE_PERIODS / targetFrequencyHertz;
		for (BaseStatusSignal signal : activeSignals) {
			if (signal.getTimestamp().getLatency() > maxAgeSeconds) {
				return false;
			}
		}
		return true;
	}

	private void recordValidSamples(ActiveBatch currentBatch, StatusCode batchStatus) {
		BaseStatusSignal[] activeSignals = currentBatch.statusSignals();
		boolean batchValid = isBatchValid(activeSignals, batchStatus);
		if (batchValid && !isCANFD) {
			updateLastRecordedSignalTimestamps(activeSignals);
		}

		lock.lock();
		try {
			if (!batchValid) {
				totalFailedUpdates++;
				return;
			}

			double timestampSeconds = TimeUtil.getRealTimeSeconds();
			for (RegisteredSignal registeredSignal : currentBatch.registeredSignals()) {
				registeredSignal.sampleRecorder().accept(timestampSeconds);
			}

			validUpdatesSinceLastCycle++;
			validUpdatesInWindow++;
		} finally {
			lock.unlock();
		}
	}

}
