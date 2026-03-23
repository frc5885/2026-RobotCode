// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.turret.LaunchCalculator;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HubShiftUtil {
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  private static Timer shiftTimer = new Timer();
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  private static final double minFuelCountDelay = 1.0;
  private static final double maxFuelCountDelay = 1.0;
  private static final double shiftEndFuelCountExtension = 3.0;
  private static final double minTimeOfFlight = LaunchCalculator.getMinTimeOfFlight();
  private static final double maxTimeOfFlight = LaunchCalculator.getMaxTimeOfFlight();
  private static final double approachingActiveFudge = -1 * (minTimeOfFlight + minFuelCountDelay);
  private static final double endingActiveFudge =
      shiftEndFuelCountExtension + -1 * (maxTimeOfFlight + maxFuelCountDelay);

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

  // Precomputed shifted start/end times for reuse to avoid per-call allocations.
  private static final double[] shiftedStartsActive = {
    0.0,
    10.0,
    35.0 + endingActiveFudge,
    60.0 + approachingActiveFudge,
    85.0 + endingActiveFudge,
    110.0 + approachingActiveFudge
  };
  private static final double[] shiftedEndsActive = {
    10.0,
    35.0 + endingActiveFudge,
    60.0 + approachingActiveFudge,
    85.0 + endingActiveFudge,
    110.0 + approachingActiveFudge,
    140.0
  };
  private static final double[] shiftedStartsInactive = {
    0.0,
    10.0 + endingActiveFudge,
    35.0 + approachingActiveFudge,
    60.0 + endingActiveFudge,
    85.0 + approachingActiveFudge,
    110.0
  };
  private static final double[] shiftedEndsInactive = {
    10.0 + endingActiveFudge,
    35.0 + approachingActiveFudge,
    60.0 + endingActiveFudge,
    85.0 + approachingActiveFudge,
    110.0,
    140.0
  };

  // Will add allieance win override if needed
  // @Setter private static Supplier<Optional<Boolean>> allianceWinOverride = () ->
  // Optional.empty();

  public static Optional<Boolean> getAllianceWinOverride() {
    // return allianceWinOverride.get();
    return Optional.empty();
  }

  @AutoLogOutput(key = "HubShift/FirstActiveAlliance")
  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Return override value
    var winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // Return default value
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the begining of teleop. */
  public static void initialize() {
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double currentTime = shiftTimer.get();
    double stateTimeElapsed = shiftTimer.get();
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }
    ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    return shiftInfo;
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    // Choose precomputed arrays based on whether the second shift starts active
    double[] starts = shiftSchedule[1] ? shiftedStartsActive : shiftedStartsInactive;
    double[] ends = shiftSchedule[1] ? shiftedEndsActive : shiftedEndsInactive;
    return getShiftInfo(shiftSchedule, starts, ends);
    // }
  }

  public static void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Update from HubShiftUtil
    ShiftInfo official = HubShiftUtil.getOfficialShiftInfo();
    ShiftInfo shifted = HubShiftUtil.getShiftedShiftInfo();
    Logger.recordOutput("HubShift/Official", official);
    Logger.recordOutput("HubShift/Shifted", shifted);
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(shifted.remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", shifted.active());
    SmartDashboard.putString("Shifts/Game State", shifted.currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());
    SmartDashboard.putBoolean(
        "Shifts/FMS Data Received?", DriverStation.getGameSpecificMessage().length() > 0);
  }
}
