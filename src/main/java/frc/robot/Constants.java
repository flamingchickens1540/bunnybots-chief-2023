// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Drivetrain{
    public static final double[] offsets = new double[]{
//            .917725 * 360, // Module 1
            75.469 + 108 - 180 - 3 - 27 + 90,
            91.318 - 90,  // Module 2
//            .224365 * 360, // Module 3
            259.729 + 30 - 180 - 1 -27 + 90,
//            .022949 * 360,     // Module 4
            9.0 + 30 -27 + 90,
            239.766, // Module 5
            32.08,  // Module 6
//            -.423096 * 360, // Module 7
            27.861 + 120 - 27 + 90,
            105.011 + 3  // Module 8
    };


    public static final double[] cornerOffsets = new double[]{
            90, // Front Left
            90, // Front Right
            180, // Back Left
            90 // Back Right
    };



  }

  public static class Intake {
    public static int rollerID = 18;
    public static int leadPivotID = 17;
    public static int followerPivotID = 16;
  }
}
