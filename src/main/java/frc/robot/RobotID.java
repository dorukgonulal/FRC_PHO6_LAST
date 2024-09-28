package frc.robot;



public class RobotID {

    public static final class Drivebase {

        /* Front Left Module - Module 0 Sari*/
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 1;
            public static final int CANCODER_ID = 9;
        }
        /* Front Right Module - Module 1 Kırmızı */ 
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int CANCODER_ID = 10;
        }

        /* Back Left Module - Module 2 Yesil*/
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 11;
        }
        /* Back Right Module - Module 3 Gri*/
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 12;
        }

        public static final int PIGEON_ID = 13;

    }


    public static final class Intake {
        
        public static final int TOP_ROLLER_ID = 17;
        public static final int BOTTOM_ROLLER_ID = 20;
        public static final int BACK_ROLLER_ID = 16;
        
    }

    public static final class Pivot {
        public static final int PIVOT_MOTOR_ID = 52;
    }

    public static final class Elevator {
        public static final int ELEVATOR_LEFT_MOTOR = 19;
        public static final int ELEVATOR_RIGHT_MOTOR = 18;
    }
    

}