// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.units.measure.Frequency;
import java.util.function.Supplier;

public class PhoenixUtil {
  private static StatusSignalCollection signals = new StatusSignalCollection();

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /**
   * Registers status signals with the specified frequency.
   * @param frequency The frequency to set the status signals to.
   * @param signals The status signals to register.
   */
  public static void registerStatusSignals(Frequency frequency, BaseStatusSignal... signals) {
    BaseStatusSignal.setUpdateFrequencyForAll(frequency, signals);
    PhoenixUtil.registerStatusSignals(signals);
  }

  /**
   * Registers status signals with the specified frequency.
   * @param signals The status signals to register.
   */
  public static void registerStatusSignals(BaseStatusSignal... signals) {
    PhoenixUtil.signals.addSignals(signals);
  }

  /**
   * Refreshes all registered status signals.
   * @return The status code of the refresh operation.
   */
  public static StatusCode refreshAll() {
    return PhoenixUtil.signals.refreshAll();
  }
}
