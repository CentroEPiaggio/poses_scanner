#include <math.h>

#define NUM_OF_SENSORS 3
#define NUM_OF_MOTORS 2
#define PI 3.14159265359

#define DEFAULT_RESOLUTION 1
#define DEFAULT_INF_LIMIT -65536.0
#define DEFAULT_SUP_LIMIT 65536.0
#define BROADCAST_ID 0
#define DEFAULT_PID_P 0.001
#define DEFAULT_PID_I 0
#define DEFAULT_PID_D 0.7
#define DEFAULT_INCREMENT 1 //in degree
#define DEFAULT_STIFFNESS 35 //in degree
#define DEFAULT_MAX_EXCURSION 360 //in degree

#define DEG_TICK_MULTIPLIER (65536.0 / (360.0 * (pow(2, DEFAULT_RESOLUTION))))

#define SIN_FILE "./conf_files/sin.conf"
#define MOTOR_FILE "./conf_files/motor.conf"
#define QBMOVE_FILE "./conf_files/qbmove.conf"
#define OLD_QBBACKUP_FOLDER "./../old_qb_backup/"
#define NEW_QBBACKUP_FOLDER "./../new_qb_backup/"
