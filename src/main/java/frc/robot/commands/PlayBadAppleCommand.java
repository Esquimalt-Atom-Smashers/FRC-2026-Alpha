package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.bad_apple.BadAppleFrame;
import frc.robot.bad_apple.BadApplePixel;

import org.littletonrobotics.junction.Logger;
import java.util.concurrent.TimeUnit;
/**
 * While scheduled: when Shooter is ready (Turret aimed, Flywheel at speed, (Optional) Hood at target),
 * runs Transfer then 0.25 s later Agitator (shooting mode). Idles both on end/cancel.
 */
public class PlayBadAppleCommand extends Command {
  BadAppleFrame[] frames;
  public PlayBadAppleCommand(BadAppleFrame[] frames) {
    this.frames = frames;
  } // End ShootWhenReadyCommand Constructor

  @Override
  public void initialize() {
    Logger.recordOutput("BadApple/CommandInitiated", true);
  } // End initialize

  @Override
  public void execute() {
    for (BadAppleFrame frame : frames) {
      printFrame(frame);
      
      try {
          TimeUnit.MILLISECONDS.sleep(100);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  } // End execute

  @Override
  public void end(boolean interrupted) {
  } // End end

  private void printFrame(BadAppleFrame frame) {
    for (BadApplePixel pixel : frame.pixels) {
      Logger.recordOutput("BadApple/Pixels/x" + pixel.x + "y" + pixel.y, pixel.getColorState());
    }
  }

  @Override
  public boolean isFinished() {
    return false; // runs until cancelled (toggle off)
  } // End isFinished
}
