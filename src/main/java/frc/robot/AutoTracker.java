package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoTracker {
    public static boolean[] tracker = {
        false,
        false
    };

    public static enum tracked {
        PATH(0),
        INTAKE(1);

        public int index;

        private tracked(int index) {
            this.index = index;
        }
    }

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
