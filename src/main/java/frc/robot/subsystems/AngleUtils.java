// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Everything in here is currently unused */

public class AngleUtils {

    public static double normalizeRotations(double rotations) {
        while (rotations < 0 || rotations > 1) {
            rotations += rotations < 0 ? 1 : -1;
        }
        return rotations;
    }

    public static double optimizeRotations(double rotations) {
        rotations = normalizeRotations(rotations);
        if (rotations > 0.5) {
            rotations = rotations - 1;
        }
        return rotations;
    }
}
