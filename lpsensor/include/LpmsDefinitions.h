/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_DEFINITIONS
#define LPMS_DEFINITIONS

/* Available device types:*/
#define DEVICE_LPMS_B       0 // LPMS-B (Bluetooth)
#define DEVICE_LPMS_U       1 // LPMS-CU / LPMS-USBAL (USB)
#define DEVICE_LPMS_C       2 // LPMS-CU / LPMS-CANAL(CAN bus)
#define DEVICE_LPMS_BLE     3 // LPMS-BLE (Bluetooth low energy)
#define DEVICE_LPMS_RS232   4 // LPMS-UARTAL (RS-232)
#define DEVICE_LPMS_B2      5 // LPMS-B2
#define DEVICE_LPMS_U2      6 // LPMS-CU2/URS2/UTTL2/USBAL2 (USB)
#define DEVICE_LPMS_C2      7 // LPMS-CU2/CANAL2 (CAN)

#define SENSOR_STATUS_ERROR         3
#define SENSOR_STATUS_CALIBRATING   2
#define SENSOR_STATUS_RUNNING       1
#define SENSOR_STATUS_PAUSED        0
#define SENSOR_STATUS_UPLOADING     4

#define SENSOR_CONNECTION_INTERRUPTED   4
#define SENSOR_CONNECTION_FAILED        3
#define SENSOR_CONNECTION_CONNECTING    2
#define SENSOR_CONNECTION_CONNECTED     1

/* Supported sensor parameters. For set / get with CalibrationData class. */
#define PRM_NAME                    0 // Sensor name (string)
#define PRM_DEVICE_ID               1 // Sensor device ID (int)
#define PRM_GYR_BIAS                2 // Gyroscope bias (Eigen::Vector3f)
#define PRM_MAG_BIAS                3 // Magnetometer bias (Eigen::Vector3f)
#define PRM_GYR_THRESHOLD           4 // Gyroscope threshold (Eigen::Vector3f)
#define PRM_MAG_THRESHOLD           5 // Magnetometer threshold (int)
#define PRM_ACC_REFERENCE           6 // Accelerometer reference (Eigen::Vector3f)
#define PRM_MAG_REFERENCE           7 // Magnetometer reference (Eigen::Vector3f)
#define PRM_OPENMAT_ID              8 // OpenMAT ID (int)
#define PRM_DEVICE_TYPE             9 // Device type (int)
#define PRM_GYR_THRESHOLD_ENABLED   10 // Gyroscope threshold enable / disable (int)
#define PRM_PARAMETER_SET           11 // Sensor filter parameter set (int)
#define PRM_FILTER_MODE             12 // Sensor filter mode (int)
#define PRM_GYR_RANGE               13 // Gyroscope range (int)
#define PRM_MAG_RANGE               14 // Magnetometer range (int)
#define PRM_ACC_RANGE               15 // Accelerometer range (int)
#define PRM_SAMPLING_RATE           16 // Sampling rate to be set (int)
#define PRM_LOCAL_Q                 17 // Enable / disable local quaternion calculation (int)
#define PRM_ACC_COVARIANCE          18 // Accelerometer covariance (float)
#define PRM_MAG_COVARIANCE          19 // Magnetometer covariance (float)
#define PRM_ACC_GAIN                20 // Accelerometer filter gain (float)
#define PRM_MAG_GAIN                21 // Magnetometer filter gain (float)
#define PRM_OFFSET_Q                22 // Local offset quaternion (Eigen::Vector4f)
#define PRM_MAG_AUTOCALIBRATION     23 // Local offset quaternion (Eigen::Vector4f)
#define PRM_CAN_STREAM_FORMAT       24 // CAN streaming format
#define PRM_CAN_BAUDRATE            25 // CAN baudrate
#define PRM_SELF_TEST               26 // Selftest enable / disable
#define PRM_GYR_AUTOCALIBRATION     27 // Gyroscope autocalibration enable / disable
#define PRM_SELECT_DATA             28 // Select stream data
#define PRM_FIRMWARE_VERSION        29 // Firmware version
#define PRM_LOW_PASS                30 // Low-pass filter setting
#define PRM_CAN_MAPPING             31 // CANopen mapping
#define PRM_CAN_HEARTBEAT           32 // CANopen heartbeat timing
#define PRM_HEAVEMOTION_ENABLED     33 // Heave motion setting
#define PRM_LIN_ACC_COMP_MODE       34 // Linear acceleration compensation mode
#define PRM_CENTRI_COMP_MODE        35 // Centripetal acceleration compensation mode
#define PRM_CAN_CHANNEL_MODE        36
#define PRM_CAN_POINT_MODE          37
#define PRM_CAN_START_ID            38
#define PRM_GAIT_TRACKING_ENABLED   39
#define PRM_LPBUS_DATA_MODE         40
#define PRM_UART_BAUDRATE           41
#define PRM_UART_FORMAT             42

/* Sensor fusion modes. */
#define SELECT_FM_GYRO_ONLY         0
#define SELECT_FM_GYRO_ACC          1
#define SELECT_FM_GYRO_ACC_MAG      2
#define SELECT_FM_MADGWICK_GYRO_ACC 3
#define SELECT_FM_MADGWICK_GYRO_ACC_MAG 4

/* Available device types. */
#define SELECT_DEVICE_LPMS_B    0
#define SELECT_DEVICE_LPMS_U    1
#define SELECT_DEVICE_LPMS_C    2

/* LpFilter parameter sets. */
#define SELECT_IMU_SLOW     0
#define SELECT_IMU_MEDIUM   1
#define SELECT_IMU_FAST     2
#define SELECT_IMU_DYNAMIC  3   

/* Enable / disable gyrsocope threshold. */
#define SELECT_IMU_GYR_THRESH_DISABLED  0
#define SELECT_IMU_GYR_THRESH_ENABLED   1

/* Enable / disable local filtering. */
#define SELECT_IMU_LOCAL_FILTER     0
#define SELECT_IMU_REMOTE_FILTER    1

/* Data transmission modes. */
#define SELECT_LPMS_MODE_STREAM     0
#define SELECT_LPMS_MODE_COMMAND    1
#define SELECT_LPMS_MODE_SLEEP      2

/* Gyroscope ranges. */
#define SELECT_GYR_RANGE_125DPS     125
#define SELECT_GYR_RANGE_245DPS     245
#define SELECT_GYR_RANGE_250DPS     250
#define SELECT_GYR_RANGE_500DPS     500 
#define SELECT_GYR_RANGE_1000DPS    1000 
#define SELECT_GYR_RANGE_2000DPS    2000

/* Accelerometer ranges. */
#define SELECT_ACC_RANGE_2G     2
#define SELECT_ACC_RANGE_4G     4
#define SELECT_ACC_RANGE_8G     8
#define SELECT_ACC_RANGE_16G    16

/* Magnetometer ranges. */
#define SELECT_MAG_RANGE_130UT      130
#define SELECT_MAG_RANGE_190UT      190
#define SELECT_MAG_RANGE_250UT      250
#define SELECT_MAG_RANGE_400UT      400
#define SELECT_MAG_RANGE_470UT      470
#define SELECT_MAG_RANGE_560UT      560
#define SELECT_MAG_RANGE_810UT      810


#define SELECT_MAG_RANGE_4GAUSS     4
#define SELECT_MAG_RANGE_8GAUSS     8
#define SELECT_MAG_RANGE_12GAUSS    12
#define SELECT_MAG_RANGE_16GAUSS    16


/* Enable / disable autocalibration. */
#define SELECT_MAG_AUTOCALIBRATION_DISABLED     0
#define SELECT_MAG_AUTOCALIBRATION_ENABLED      1

/* CAN bus protocol. */
#define SELECT_STREAM_CAN_LPBUS         0
#define SELECT_STREAM_CAN_CUSTOM1       1
#define SELECT_STREAM_CAN_OPEN          2
#define SELECT_STREAM_CAN_CUSTOM2       3
#define SELECT_STREAM_CAN_CUSTOM3       4

/* CAN bus baudrate. */
#define SELECT_CAN_BAUDRATE_125KBPS     125
#define SELECT_CAN_BAUDRATE_250KBPS     250
#define SELECT_CAN_BAUDRATE_500KBPS     500
#define SELECT_CAN_BAUDRATE_1000KBPS    1000

/* Self-test on / off. */
#define SELECT_SELF_TEST_ON             0
#define SELECT_SELF_TEST_OFF            1

/* Data streaming frequency. */
#define SELECT_STREAM_FREQ_5HZ          5
#define SELECT_STREAM_FREQ_10HZ         10
#define SELECT_STREAM_FREQ_25HZ         25
#define SELECT_STREAM_FREQ_50HZ         50
#define SELECT_STREAM_FREQ_100HZ        100
#define SELECT_STREAM_FREQ_200HZ        200
#define SELECT_STREAM_FREQ_400HZ        400
#define SELECT_STREAM_FREQ_800HZ        800

/* Magnetometer threshold level. */
#define SELECT_MAG_THRESH_DISABLE       0
#define SELECT_MAG_THRESH_5UT           1
#define SELECT_MAG_THRESH_10UT          2
#define SELECT_MAG_THRESH_20UT          3

/* Gyroscope auto-calibration on / off. */
#define SELECT_GYR_AUTOCALIBRATION_DISABLED     0
#define SELECT_GYR_AUTOCALIBRATION_ENABLED      1

/* Magnetic field map dimensions. */
#define ABSMAXPITCH 3
#define ABSMAXROLL 6
#define ABSMAXYAW 6

/* Data output selection. */
#define SELECT_LPMS_QUAT_OUTPUT_ENABLED                 0x1 
#define SELECT_LPMS_EULER_OUTPUT_ENABLED                (0x1 << 1)
#define SELECT_LPMS_LINACC_OUTPUT_ENABLED               (0x1 << 2)
#define SELECT_LPMS_PRESSURE_OUTPUT_ENABLED             (0x1 << 3)
#define SELECT_LPMS_GYRO_OUTPUT_ENABLED                 (0x1 << 4)
#define SELECT_LPMS_ACC_OUTPUT_ENABLED                  (0x1 << 5)
#define SELECT_LPMS_MAG_OUTPUT_ENABLED                  (0x1 << 6)
#define SELECT_LPMS_GYRO_TEMP_OUTPUT_ENABLED            (0x1 << 7)
#define SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED          (0x1 << 8)
#define SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED             (0x1 << 9)
#define SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED     (0x1 << 10)
#define SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED          (0x1 << 11)

#define SELECT_LPMS_LP_OFF                      0
#define SELECT_LPMS_LP_01                       1
#define SELECT_LPMS_LP_005                      2
#define SELECT_LPMS_LP_001                      3
#define SELECT_LPMS_LP_0005                     4
#define SELECT_LPMS_LP_0001                     5

#define SELECT_LPMS_CAN_HEARTBEAT_005           0
#define SELECT_LPMS_CAN_HEARTBEAT_010           1
#define SELECT_LPMS_CAN_HEARTBEAT_020           2
#define SELECT_LPMS_CAN_HEARTBEAT_050           3
#define SELECT_LPMS_CAN_HEARTBEAT_100           4

#define SELECT_HEAVEMOTION_DISABLED             0
#define SELECT_HEAVEMOTION_ENABLED              1

#define SELECT_GAIT_TRACKING_ENABLED            0
#define SELECT_GAIT_TRACKING_DISABLED           1

#define SELECT_LPMS_LIN_ACC_COMP_MODE_OFF       0
#define SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK      1
#define SELECT_LPMS_LIN_ACC_COMP_MODE_MEDIUM    2
#define SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG    3
#define SELECT_LPMS_LIN_ACC_COMP_MODE_ULTRA     4

#define SELECT_LPMS_CENTRI_COMP_MODE_OFF        0
#define SELECT_LPMS_CENTRI_COMP_MODE_ON         1

#define SELECT_CAN_POINT_MODE_FIXED             1
#define SELECT_CAN_POINT_MODE_FLOATING          0

#define SELECT_CAN_CHANNEL_MODE_SEQUENTIAL      1
#define SELECT_CAN_CHANNEL_MODE_CANOPEN         0

#define SELECT_LPMS_LPBUS_DATA_MODE_32          0
#define SELECT_LPMS_LPBUS_DATA_MODE_16          1

#define SELECT_LPMS_UART_BAUDRATE_19200         0
#define SELECT_LPMS_UART_BAUDRATE_57600         1
#define SELECT_LPMS_UART_BAUDRATE_115200        2
#define SELECT_LPMS_UART_BAUDRATE_921600        3

#define SELECT_LPMS_UART_FORMAT_LPBUS           0
#define SELECT_LPMS_UART_FORMAT_CSV             1

#endif