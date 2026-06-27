package frc.robot.util;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MultiTurnAbsoluteEncoder {

  private final DoubleSupplier absRotSource;
  private final BooleanSupplier connectedSource;

  private boolean hasPrev = false;
  private double prevAbsRot = 0.0;
  private int turns = 0;

  /** REV Through-Bore (or any duty-cycle absolute encoder) wired to a RoboRIO DIO channel. */
  public MultiTurnAbsoluteEncoder(int dioChannel) {
    DutyCycleEncoder encoder = new DutyCycleEncoder(dioChannel);
    this.absRotSource = encoder::get;
    this.connectedSource = encoder::isConnected;
  }

  /**
   * Any absolute source reporting rotations in [0,1) — e.g. a Spark MAX data-port absolute
   * encoder via {@code sparkMax.getAbsoluteEncoder()::getPosition}. Same wrap-tracking logic.
   */
  public MultiTurnAbsoluteEncoder(DoubleSupplier absRotSource, BooleanSupplier connectedSource) {
    this.absRotSource = absRotSource;
    this.connectedSource = connectedSource;
  }

  public boolean isConnected() {
    return connectedSource.getAsBoolean();
  }

  // Returns absolute within one turn, rotations in range [0,1)
  public double getAbsRot() {
    return absRotSource.getAsDouble();
  }

  // Returns continuous rotations, can exceed 1.0 or be negative
  public double getContinuousRot() {
    double abs = getAbsRot();

    if (!hasPrev) {
      hasPrev = true;
      prevAbsRot = abs;
      return turns + abs;
    }

    double delta = abs - prevAbsRot;

    // Wrap detection for crossing 0 boundary
    if (delta > 0.5) {
      turns -= 1;
    } else if (delta < -0.5) {
      turns += 1;
    }

    prevAbsRot = abs;
    return turns + abs;
  }

  public void resetToCurrentAsZero() {
    hasPrev = false;
    turns = 0;
  }
}