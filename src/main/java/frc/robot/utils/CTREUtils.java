package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DataLogManager;

public class CTREUtils {
    private CTREUtils() {}

    public static void checkCTREError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DataLogManager.log(String.format("%s: %s", message, errorCode.toString()));
        }
    }
}
