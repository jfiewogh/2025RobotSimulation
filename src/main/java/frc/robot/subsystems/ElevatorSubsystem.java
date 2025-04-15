// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends SubsystemBase {

  StructPublisher<Pose3d> element = NetworkTableInstance.getDefault()
    .getStructTopic("MyThing", Pose3d.struct).publish();

  Pose3d pose = new Pose3d(0, 0, 0, Rotation3d.kZero);

  Mechanism2d mechanism = new Mechanism2d(2, 4);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    mechanism.getRoot("this", 0.2, 0).append(new MechanismLigament2d("extend", 1, 90));
    mechanism.getRoot("other", -0.2, 0).append(new MechanismLigament2d("expand", 1, 90));
  }

  public Command goUp() {
    return new InstantCommand(
      () -> {
        System.out.println("go up");
        pose = new Pose3d(pose.getX(), pose.getY() + 0.5, pose.getZ(), Rotation3d.kZero);

        // pose.transformBy(new Transform3d(0, 3, 0, Rotation3d.kZero));
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    element.set(pose);
  }
}
