package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),
  AMP_5_PIECE("Red Amp 5 Piece", ""),
  SOURCE_4_PIECE("Red Source 4 Piece", ""),
  OP("Red OP", "");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}
