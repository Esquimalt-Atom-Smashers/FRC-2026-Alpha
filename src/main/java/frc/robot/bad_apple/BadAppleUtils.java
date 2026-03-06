package frc.robot.bad_apple;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.Java2DFrameConverter;

import edu.wpi.first.wpilibj.util.Color;

public class BadAppleUtils {

    // Convert a video into a sequence of small BufferedImages
    public static BufferedImage[] convertVideo(String path, int grabEveryFrame) throws Exception {
        final int WIDTH = 16;  // desired width of output frame
        final int HEIGHT = 12; // desired height of output frame

        ArrayList<BufferedImage> frames = new ArrayList<>();

        FFmpegFrameGrabber grabber = new FFmpegFrameGrabber(path);
        Java2DFrameConverter converter = new Java2DFrameConverter();

        grabber.start();

        Frame frame;
        int frameIndex = 0;

        while ((frame = grabber.grabImage()) != null) {
            if (frameIndex++ % grabEveryFrame != 0) continue;

            BufferedImage img = converter.convert(frame);

            // resize to WIDTH x HEIGHT using binary image type
            BufferedImage resized = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_BYTE_BINARY);
            Graphics2D g = resized.createGraphics();
            g.drawImage(img, 0, 0, WIDTH, HEIGHT, null);
            g.dispose();

            frames.add(resized);
        }

        grabber.stop();
        grabber.close();

        return frames.toArray(new BufferedImage[0]);
    }

    // Convert a single BufferedImage into an array of BadApplePixel
    public static BadApplePixel[] convertTo2DUsingGetRGB(BufferedImage image) {
        int width = image.getWidth();
        int height = image.getHeight();

        ArrayList<BadApplePixel> pixels = new ArrayList<>();

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int clr = image.getRGB(x, y);
                int red =   (clr & 0x00ff0000) >> 16;
                int green = (clr & 0x0000ff00) >> 8;
                int blue =  clr & 0x000000ff;

                pixels.add(new BadApplePixel(x, y, new Color(red, green, blue)));
            }
        }

        return pixels.toArray(new BadApplePixel[0]);
    }

    // Print a single frame to console as '#' (black) and ' ' (white)
    public static void printFrame(BufferedImage frame) {
        int width = frame.getWidth();
        int height = frame.getHeight();

        for (int y = 0; y < height; y++) {
            StringBuilder sb = new StringBuilder();
            for (int x = 0; x < width; x++) {
                int clr = frame.getRGB(x, y) & 0xFFFFFF; // ignore alpha
                sb.append(clr == 0x000000 ? "#" : " "); // black = '#', white = ' '
            }
            System.out.println(sb.toString());
        }
    }
}