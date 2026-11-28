package frc.robot.hardware.phoenix6.signal.threaded;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Robot;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6Util;
import frc.utils.AngleUnit;

public class Phoenix6ThreadSignalBuilder {

	private static final int UPDATE_FREQUENCY_RETRIES = 5;

	private static void setFrequencyWithRetry(StatusSignal<?> signal, double frequencyHertz) {
		if (!Robot.ROBOT_TYPE.isReplay()) {
			Phoenix6Util.checkStatusCodeWithRetry(() -> signal.setUpdateFrequency(frequencyHertz), UPDATE_FREQUENCY_RETRIES);
		}
	}

	private static StatusSignal<?> cloneWithFrequency(StatusSignal<?> signal, double frequencyHertz) {
		StatusSignal<?> signalClone = signal.clone();
		setFrequencyWithRetry(signalClone, frequencyHertz);
		return signalClone;
	}

	private static void refreshIfActive(BaseStatusSignal... signals) {
		if (!Robot.ROBOT_TYPE.isReplay()) {
			BaseStatusSignal.refreshAll(signals);
		}
	}

	public static ThreadedDoubleSignal build(StatusSignal<?> signal, double frequencyHertz, BusChain busChain) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequencyHertz);
		refreshIfActive(signalClone);
		Phoenix6SignalThread thread = Phoenix6SignalThread.getOrCreate(busChain);
		SignalSampleQueue queue = thread.createQueue();
		ThreadedDoubleSignal threadedSignal = new ThreadedDoubleSignal(signalClone.getName(), signalClone, queue);
		thread.registerSignal(queue, threadedSignal::recordSample, frequencyHertz, signalClone);
		return threadedSignal;
	}

	public static ThreadedAngleSignal build(StatusSignal<?> signal, double frequencyHertz, AngleUnit angleUnit, BusChain busChain) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequencyHertz);
		refreshIfActive(signalClone);
		Phoenix6SignalThread thread = Phoenix6SignalThread.getOrCreate(busChain);
		SignalSampleQueue queue = thread.createQueue();
		ThreadedAngleSignal threadedSignal = new ThreadedAngleSignal(signalClone.getName(), signalClone, angleUnit, queue);
		thread.registerSignal(queue, threadedSignal::recordSample, frequencyHertz, signalClone);
		return threadedSignal;
	}

	public static ThreadedLatencySignal build(
		StatusSignal<?> signal,
		ThreadedAngleSignal signalSlope,
		double frequencyHertz,
		AngleUnit angleUnit,
		BusChain busChain
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequencyHertz);
		StatusSignal<?> slopeSignal = signalSlope.getSignal();
		setFrequencyWithRetry(slopeSignal, frequencyHertz);
		refreshIfActive(signalClone, slopeSignal);
		Phoenix6SignalThread thread = Phoenix6SignalThread.getOrCreate(busChain);
		SignalSampleQueue queue = thread.createQueue();
		ThreadedLatencySignal threadedSignal = new ThreadedLatencySignal(signalClone.getName(), signalClone, slopeSignal, angleUnit, queue);
		thread.registerSignal(queue, threadedSignal::recordSample, frequencyHertz, signalClone, slopeSignal);
		return threadedSignal;
	}

	public static ThreadedLatencySignal build(
		StatusSignal<?> signal,
		StatusSignal<?> signalSlope,
		double frequencyHertz,
		AngleUnit angleUnit,
		BusChain busChain
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequencyHertz);
		StatusSignal<?> slopeClone = cloneWithFrequency(signalSlope, frequencyHertz);
		refreshIfActive(signalClone, slopeClone);
		Phoenix6SignalThread thread = Phoenix6SignalThread.getOrCreate(busChain);
		SignalSampleQueue queue = thread.createQueue();
		ThreadedLatencySignal threadedSignal = new ThreadedLatencySignal(signalClone.getName(), signalClone, slopeClone, angleUnit, queue);
		thread.registerSignal(queue, threadedSignal::recordSample, frequencyHertz, signalClone, slopeClone);
		return threadedSignal;
	}

}
