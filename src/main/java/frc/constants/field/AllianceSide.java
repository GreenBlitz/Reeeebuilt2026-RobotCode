package frc.constants.field;

public enum AllianceSide {

	DEPOT,
	OUTPOST;

	public AllianceSide getOppositeSide() {
		return switch (this) {
			case DEPOT -> OUTPOST;
			case OUTPOST -> DEPOT;
		};
	}

}
