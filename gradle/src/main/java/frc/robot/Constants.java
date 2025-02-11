// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String RobotName = "";
  public static final int kControllerPort = 0;

  public static final String MAVEN_GROUP = "";
  public static final String MAVEN_NAME = "Kod 0.4";
  public static final String VERSION = "0.4";
  public static final int GIT_REVISION = -1;
  public static final String GIT_SHA = "UNKNOWN";
  public static final String GIT_DATE = "UNKNOWN";
  public static final String GIT_BRANCH = "UNKNOWN";
  public static final String BUILD_DATE = "2025-01-21 20:27:14 EST";
  public static final long BUILD_UNIX_TIME = 1737509234319L;
  public static final int DIRTY = 129;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
