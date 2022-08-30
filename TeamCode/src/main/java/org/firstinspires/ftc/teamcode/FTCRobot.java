package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;

public class FTCRobot extends Robot {
    // enum to specify opmode type
    public enum OpModeType {
        TELEOP, AUTO
    }

    public FTCRobot (OpModeType type) {
        if (type == OpModeType.TELEOP){
            initTele();
        }else {
            initAuto();
        }
    }

    /*
     * Initialize teleop or autonomous, depending on which is used
     */
    public void initTele() {
        // initialize teleop-specific scheduler
    }

    public void initAuto() {
        // initialize auto-specific scheduler
    }
}
