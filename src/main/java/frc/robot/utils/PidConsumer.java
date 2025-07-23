package frc.robot.utils;
@FunctionalInterface
public interface PidConsumer {
    public void accept(double kP, double kI, double kD);
         
    
}
