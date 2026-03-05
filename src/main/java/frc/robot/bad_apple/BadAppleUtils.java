package frc.robot.bad_apple;

import org.bytedeco.javacv.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.util.Color;

public class BadAppleUtils {
  public static BufferedImage[] convertVideo(String path, int grabEveryFrame) throws Exception {
    ArrayList<BufferedImage> frames = new ArrayList<>();

    FFmpegFrameGrabber grabber = new FFmpegFrameGrabber(path);
    Java2DFrameConverter converter = new Java2DFrameConverter();

    grabber.start();

    Frame frame;
    while ((frame = grabber.grabImage()) != null) {
      BufferedImage img = converter.convert(frame);
      frames.add(img);
    }

    grabber.stop();

    return frames.toArray(new BufferedImage[0]);
  }
  
  public static BadApplePixel[] convertTo2DUsingGetRGB(BufferedImage image) {
     int width = image.getWidth();
     int height = image.getHeight();
     ArrayList<BadApplePixel> pixels = new ArrayList<>();

     for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
          int clr = image.getRGB(col, row);
          int red =   (clr & 0x00ff0000) >> 16;
          int green = (clr & 0x0000ff00) >> 8;
          int blue =   clr & 0x000000ff;

          pixels.add(new BadApplePixel(col, row, new Color(red, green, blue)));
        }
     }

     return pixels.toArray(new BadApplePixel[0]);
  }
}
