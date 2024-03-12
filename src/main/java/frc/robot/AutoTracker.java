package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoTracker {
  public static boolean[] tracker = {
      false,
      false,
      false
  };

  public static enum tracked {
      PATH(0),
      INTAKE(1),
      SHOOTER(2);
      public int index;
      private tracked(int index) {
          this.index = index;
      }
  }

  /**
   * Waits for a signal sent by sendSignal
   * @param t the id of the signal to wait for
   * @return a command that waits for the signal
   */
  public static Command waitForSignal(tracked t) {
    return new Command() {
      @Override
      public void initialize() {
        tracker[t.index] = false;
      }

      @Override
      public boolean isFinished() {
        return tracker[t.index];
      }
    };
  }
  
  /**
   * Sends a signal to the waitForSignal command
   * @param t the id of the signal to send
   * @return a command that sends the signal
   */
  public static Command sendSignal(tracked t) {
    return new Command() {
      @Override
      public void initialize() {
        tracker[t.index] = true;
      }
  
      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }
}
