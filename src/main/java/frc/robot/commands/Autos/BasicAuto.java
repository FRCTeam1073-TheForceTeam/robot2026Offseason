// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveStraight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;

public final class BasicAuto
{
  private BasicAuto()
  {
  }

  public static Command create(Drivetrain drivetrain, Localizer localizer)
  {
    return new DriveStraight(drivetrain, localizer);
  }
}
