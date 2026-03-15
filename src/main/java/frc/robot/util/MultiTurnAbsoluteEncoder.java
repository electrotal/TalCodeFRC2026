package frc.robot.util;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class MultiTurnAbsoluteEncoder {

  private final DutyCycleEncoder encoder;

  private boolean hasPrev = false;
  private double prevAbsRot = 0.0;
  private int turns = 0;

  public MultiTurnAbsoluteEncoder(int dioChannel) {
    encoder = new DutyCycleEncoder(dioChannel);
  }

  public boolean isConnected() {
    return encoder.isConnected();
  }

  // Returns absolute within one turn, rotations in range [0,1)
  public double getAbsRot() {
    return encoder.get();
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