package frc.team5115.robot;

import edu.wpi.first.wpilibj.Timer;

public class Heartbeat {

        private double start;
        private double checkpoint;
        private double difference;

        public void start(){
                start = Timer.getFPGATimestamp() * 1000;
        }

        public void check(){
                difference = Timer.getFPGATimestamp() * 1000 - checkpoint;
                checkpoint = Timer.getFPGATimestamp() * 1000;
        }

        public double getAbsolute(){
                return Timer.getFPGATimestamp() * 1000 - start;

        }

        public double getDifference(){
                return difference;
        }
}
