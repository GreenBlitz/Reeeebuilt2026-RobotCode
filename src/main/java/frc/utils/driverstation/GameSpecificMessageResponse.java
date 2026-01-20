package frc.utils.driverstation;


public enum GameSpecificMessageResponse {

	RED('R'),
	BLUE('B'),
	EMPTY(' ');

	private final char value;

	GameSpecificMessageResponse(char value) {
		this.value = value;
	}

	public char getValue() {
		return value;
	}

	public static GameSpecificMessageResponse responseToEnum(char value) {
		return switch (value) {
			case 'R' -> RED;
			case 'B' -> BLUE;
			default -> EMPTY;
		};
	}

}
