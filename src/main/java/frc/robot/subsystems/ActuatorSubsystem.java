package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ActuatorSubsystem extends SubsystemBase {
    // מחובר ליציאת PWM מספר 1 ברובוריו
    private final Servo myServo = new Servo(9);

    public ActuatorSubsystem() {
        // 0.5 מציין "עצירה" בסרבו רציף מתמשך (Continuous)
        //myServo.setAngle(45); 
    }

    // פונקציה שמקבלת ערך ומעבירה לסרבו
    public void setServo(double DUD) {
        myServo.setPosition(0.5);
    }

    @Override
    public void periodic() {
        // מדפיס לדיבאג כדי שנראה איזה פקודה הסרבו מקבל עכשיו
    }
}