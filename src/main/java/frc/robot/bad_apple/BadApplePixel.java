package frc.robot.bad_apple;

import edu.wpi.first.wpilibj.util.Color;

public class BadApplePixel {
  public BadApplePixel(int x, int y, Color color) {
    this.x = x;
    this.y = y;
    this.color = color;
  }

  public int x;
  public int y;
  public Color color;

  public static boolean getColorState(Color color) {
    return false;
  }
}
