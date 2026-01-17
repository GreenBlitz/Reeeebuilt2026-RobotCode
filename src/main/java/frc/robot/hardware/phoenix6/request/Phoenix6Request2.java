package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.hardware.interfaces.IRequest2;

import java.util.function.Consumer;

public class Phoenix6Request2<T, K> implements IRequest2<T, K> {

	private ControlRequest controlRequest;
	private Consumer<T> setSetPoint1;
	private Consumer<K> setSetPoint2;

	private T setPoint1;
	private K setPoint2;


	public Phoenix6Request2(
		T defaultSetPoint1,
		K defaultSetPoint2,
		ControlRequest controlRequest,
		Consumer<T> setSetPoint1,
		Consumer<K> setSetPoint2
	) {
		this.setPoint1 = defaultSetPoint1;
		this.setPoint2 = defaultSetPoint2;
		this.controlRequest = controlRequest;
		this.setSetPoint1 = setSetPoint1;
		this.setSetPoint2 = setSetPoint2;
	}

	public Phoenix6Request2<T, K> withSetPoint(T setPoint1, K setPoint2) {
		setSetPoint1.accept(setPoint1);
		setSetPoint2.accept(setPoint2);
		this.setPoint1 = setPoint1;
		this.setPoint2 = setPoint2;
		return this;
	}

	public T getSetPoint1() {
		return setPoint1;
	}

	public K getSetPoint2() {
		return setPoint2;
	}

	public ControlRequest getControlRequest() {
		return controlRequest;
	}

}
