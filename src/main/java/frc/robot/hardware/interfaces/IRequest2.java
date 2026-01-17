package frc.robot.hardware.interfaces;

public interface IRequest2<T, V> extends IRequest<T> {

	void withSetPoint1(T setPoint1, V setPoint2);

	void setPoint1(T setPoint1);

	void setPoint2(V setPoint2);

	T getSetPoint1();

	V getSetPoint2();

}
