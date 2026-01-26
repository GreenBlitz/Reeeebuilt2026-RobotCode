package frc.robot.hardware.empties;

import frc.robot.hardware.interfaces.IRequest;

public class EmptyRequest<T> implements IRequest<T> {

	@Override
	public IRequest<T> withSetPoint(T setPoint) {
		return this;
	}

	@Override
	public T getSetPoint() {
		return null;
	}

}
