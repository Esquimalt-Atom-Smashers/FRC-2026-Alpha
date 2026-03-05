package frc.robot.bad_apple;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.FFmpegFrameRecorder;
import org.bytedeco.javacv.FFmpegLogCallback;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.Java2DFrameConverter;
import org.bytedeco.javacv.OpenCVFrameConverter;
import static org.bytedeco.opencv.global.opencv_imgproc.resize;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avutil;
import edu.wpi.first.wpilibj.util.Color;

public class BadAppleUtils {
  public static BufferedImage[] convertVideo(String path, int grabEveryFrame) throws Exception {
  
      final int WIDTH = 16;
      final int HEIGHT = 12;
  
      ArrayList<BufferedImage> frames = new ArrayList<>();
  
      FFmpegFrameGrabber grabber = new FFmpegFrameGrabber(path);
      Java2DFrameConverter converter = new Java2DFrameConverter();
  
      grabber.start();
  
      Frame frame;
      int frameIndex = 0;
  
      while ((frame = grabber.grabImage()) != null) {
      
          if (frameIndex++ % grabEveryFrame != 0) {
              continue;
          }
        
          BufferedImage img = converter.convert(frame);
        
          // resize
          BufferedImage resized = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_BYTE_BINARY);
          Graphics2D g = resized.createGraphics();
        
          g.drawImage(img, 0, 0, WIDTH, HEIGHT, null);
          g.dispose();
        
          frames.add(resized);
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
