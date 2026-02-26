package frc.robot.subsystems.led;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.IDs;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.GBSubsystem;

public class CANdleWrapper extends GBSubsystem {

	private final CANdle led;
	private int ledLength = 64;

	public CANdleWrapper(String logPath) {
		super(logPath);
		led = new CANdle(IDs.CANdleIDs.LEDS, new CANBus(BusChain.ROBORIO.name()));
		configureCANdle();
	}

	private void configureCANdle() {
		CANdleConfiguration config = new CANdleConfiguration();
		config.LED.StripType = StripTypeValue.valueOf(1);
		led.getConfigurator().apply(config);
	}

	private void setLedLength(int ledLength) {
		this.ledLength = ledLength;
	}

	public void setColor(int r, int g, int b, int w) {
		RGBWColor rgbwColor = new RGBWColor(r, g, b, w);
		SolidColor color = new SolidColor(0, ledLength).withColor(rgbwColor);
		led.setControl(color);
	}

}
