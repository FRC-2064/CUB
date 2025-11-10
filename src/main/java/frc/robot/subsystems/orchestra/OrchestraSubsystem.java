package frc.robot.subsystems.orchestra;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import java.io.File;
import org.littletonrobotics.junction.Logger;

public class OrchestraSubsystem extends SubsystemBase {
  private final Orchestra orchestra;
  private final TalonFX[] motors;

  private boolean isPlaying = false;
  private String currentSong = "None";

  public OrchestraSubsystem() {
    orchestra = new Orchestra();

    motors =
        new TalonFX[] {
          new TalonFX(
              TunerConstants.FrontLeft.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName),
          new TalonFX(
              TunerConstants.FrontLeft.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName),
          new TalonFX(
              TunerConstants.FrontRight.DriveMotorId,
              TunerConstants.DrivetrainConstants.CANBusName),
          new TalonFX(
              TunerConstants.FrontRight.SteerMotorId,
              TunerConstants.DrivetrainConstants.CANBusName),
          new TalonFX(
              TunerConstants.BackLeft.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName),
          new TalonFX(
              TunerConstants.BackLeft.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName),
          new TalonFX(
              TunerConstants.BackRight.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName),
          new TalonFX(
              TunerConstants.BackRight.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName)
        };

    for (TalonFX motor : motors) {
      orchestra.addInstrument(motor);
    }

    System.out.println("Orchestra initialized with " + motors.length + " motors");
  }

  public boolean loadMusic(String filename) {
    File musicFile = new File(Filesystem.getDeployDirectory(), "music/" + filename);

    if (!musicFile.exists()) {
      System.err.println("Music file not found: " + musicFile.getAbsolutePath());
      Logger.recordOutput("Orchestra/Error", "File not found: " + filename);
      return false;
    }

    StatusCode status = orchestra.loadMusic(musicFile.getAbsolutePath());

    if (status.isOK()) {
      currentSong = filename;
      System.out.println("Loaded music file: " + filename);
      Logger.recordOutput("Orchestra/CurrentSong", currentSong);
      return true;
    } else {
      System.err.println("Failed to load music file: " + status);
      Logger.recordOutput("Orchestra/Error", "Failed to load: " + status);
      return false;
    }
  }

  public void play() {
    orchestra.play();
    isPlaying = true;
    System.out.println("Playing: " + currentSong);
  }

  public void stop() {
    orchestra.stop();
    isPlaying = false;
    System.out.println("Stopped playback");
  }

  public void pause() {
    orchestra.pause();
    isPlaying = false;
    System.out.println("Paused playback");
  }

  public boolean isPlaying() {
    return orchestra.isPlaying();
  }

  public double getCurrentTime() {
    return orchestra.getCurrentTime();
  }

  @Override
  public void periodic() {
    isPlaying = orchestra.isPlaying();

    Logger.recordOutput("Orchestra/IsPlaying", isPlaying);
    Logger.recordOutput("Orchestra/CurrentSong", currentSong);
    Logger.recordOutput("Orchestra/CurrentTime", getCurrentTime());
  }
}
