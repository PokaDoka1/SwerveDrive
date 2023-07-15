// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class SwerveModulesConstants{

        public static final double DRIVE_KP = 0;
        public static final double DRIVE_KF = 0;

        public static final double TURN_KP = 0;

        public static final double TURN_GEAR_RATIO = 12.8;
        public static final double DRIVE_GEAR_RATIO = 10;

        public static final double WHEEL_DIAMETER_METERS = 0.1016;
    }

    public static class DriveConstants{

        public static final double WHEELBASE_WIDTH = 0.6604;
        public static final double WHEELBASE_LENGTH = 0.6604;

        public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
        public static final Translation2d REAR_LEFT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
        public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
        public static final Translation2d REAR_RIGHT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_MODULE_POSITION, FRONT_RIGHT_MODULE_POSITION, REAR_LEFT_MODULE_POSITION, REAR_RIGHT_MODULE_POSITION);

        public static final int REAR_RIGHT_DRIVE_PORT = 10;
        public static final int FRONT_RIGHT_DRIVE_PORT = 7;
        public static final int FRONT_LEFT_DRIVE_PORT = 4;
        public static final int REAR_LEFT_DRIVE_PORT = 1;

        public static final int REAR_RIGHT_TURN_PORT = 11;
        public static final int FRONT_RIGHT_TURN_PORT = 8;
        public static final int FRONT_LEFT_TURN_PORT = 5;
        public static final int REAR_LEFT_TURN_PORT = 2;

        public static final int REAR_RIGHT_ENCODER_PORT = 12;
        public static final int FRONT_RIGHT_ENCODER_PORT = 9;
        public static final int FRONT_LEFT_ENCODER_PORT = 6;
        public static final int REAR_LEFT_ENCODER_PORT = 3;

        public static final double REAR_RIGHT_ENCODER_OFFSET = 10.107010016401127;//-97.3;//178.857;//174.0234375;//11.162109375;//2.724609375;
        public static final double FRONT_RIGHT_ENCODER_OFFSET =  127.0007175492034;//-167;//-4.219;//176.1328125;//-110.830078125 ;// -111.263671875;
        public static final double FRONT_LEFT_ENCODER_OFFSET = -49.65367268041238;//-168.1;//-169.277;//-4.74609375;//-6.328125;//6-2.263671875;
        public static final double REAR_LEFT_ENCODER_OFFSET = -154.42226306232428;//-107;

        //how fast you can move forward
        public static final double MAX_TANGENTIAL_VELOCITY = 4;
        public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;

        //JVN counts = weight, gear ratio, max RPM, and it calculates your meters per second
    }
}
