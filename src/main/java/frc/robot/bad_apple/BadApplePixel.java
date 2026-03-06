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

  public boolean getColorState() {
    double threshold = 0.5; // anything below is black, above is white
    boolean isWhite = color.red > threshold && color.green > threshold && color.blue > threshold;
    return isWhite;
  }
}
