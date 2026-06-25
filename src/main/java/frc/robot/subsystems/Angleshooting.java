// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// // הייבוא המעודכן לספריית REV החדשה (עונת 2025)
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.ResetMode;
// import com.revrobotics.PersistMode;

// public class Angleshooting extends SubsystemBase {
    
//     // הגדרת בקר המנוע
//     private final SparkMax angleMotor;
//     private final int MOTOR_CAN_ID = 26;

   
//     private final double MAX_ANGLE = 90.0; // גבול עליון
//     private final double MIN_ANGLE = 0.0;  // גבול תחתון

//     public Angleshooting() {
//         // אתחול המנוע
//         angleMotor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);

//         // יצירת אובייקט הגדרות (התקן החדש של REV)
//         SparkMaxConfig config = new SparkMaxConfig();

//         // הגדרת Soft Limits למניעת חריגה מזוויות
//         config.softLimit
//             .forwardSoftLimit(MAX_ANGLE)
//             .forwardSoftLimitEnabled(true)
//             .reverseSoftLimit(MIN_ANGLE)
//             .reverseSoftLimitEnabled(true);

//         // החלת ההגדרות על הבקר באופן בטיחותי ושמירתן בזיכרון הלא-נדיף
//         angleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     // פונקציית הפעלה
//     public void setSpeed(double speed) {
//         angleMotor.set(speed);
//     }

//     // פונקציית עצירה
//     public void stop() {
//         angleMotor.set(0);
//     }

//     /**
//      * פונקציה שמייצרת Command עבור השלט.
//      * מפעילה את המנוע כל עוד הפקודה רצה, ועוצרת אותו כשהיא מסתיימת.
//      */
//     public Command runMotorCommand(double speed) {
//         return this.runEnd(
//             () -> this.setSpeed(speed), // בזמן ריצה
//             () -> this.stop()           // בסיום ריצה
//         );
//     }
// }