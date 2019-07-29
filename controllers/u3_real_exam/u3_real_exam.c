/*
 * File:          u3_real_exam_Duarte_Jesus.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
 /*default libreries*/
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

/* C LIBRARIES */
#include <stdio.h>

#define TIME_STEP 64

#define RADIUS_WHEELS 0.09
#define PI 3.14
#define MAX_BITS 255
#define SPIN_DEGREE 45
//#define angle_left 0
//#define angle_right 0

#define MAX_VELOCITY 30.36
#define VELOCITY_AUTONOMOUS -8
#define VELOCITY_MANUAL -6
#define DISTANCE_OBSTACLE 19


/* global variables */
int angle_left, angle_right = 0;
char *file1 = "Distance sensor right value:";
char *file2 = "Position sensor motor1:";
char *file3 = "Distance sensor left value:";
char *file4 = "Position sensor motor2:";
char *file5 = "Position sensor motor3:";

double new_position;
double new_position_enemy1;
double new_position_enemy2;
double new_position_enemy3;

int flg1 = 0;
int flg2 = 0;
int flg3 = 0;

/*functions*/
float resolutionToCentimeters(float centimeters) {
    return (MAX_BITS*centimeters)/(40);
}

void stopMotors(WbDeviceTag first_motor, WbDeviceTag second_motor,
                   WbDeviceTag third_motor) {
    wb_motor_set_velocity(first_motor, 0);
    wb_motor_set_velocity(second_motor, 0);
    wb_motor_set_velocity(third_motor, 0);
}

float angularToLinealVelocity(float meters_per_second) {
    float RPM;
    float linear_velocity;

    meters_per_second = meters_per_second / RADIUS_WHEELS;
    RPM = (meters_per_second * 290) / MAX_VELOCITY;
    linear_velocity = ((2 * PI * RADIUS_WHEELS) / 60) * RPM;

    return linear_velocity;
}

float degreesSec2RadSec(void) {
    float rad_sec;

    rad_sec = SPIN_DEGREE* 0.017452;

    return rad_sec;
}

float rpm2Radians(float radians) {
    int integer_part;
    float result;
    float decimal_part;
    float turns;

    integer_part = radians/(2*PI);
    decimal_part = radians/(2*PI);

    turns = decimal_part - integer_part;

    result = turns * (2*PI);

    return result;
}

void manual(int key, WbDeviceTag first_motor, WbDeviceTag second_motor,
            WbDeviceTag third_motor) {
    switch (key) {

        case WB_KEYBOARD_UP:
             wb_motor_set_velocity(first_motor,VELOCITY_MANUAL);
             wb_motor_set_velocity(second_motor,-VELOCITY_MANUAL);
             wb_motor_set_velocity(third_motor,0);
             printf("Linear Velocity is: %.2lf\n",
             angularToLinealVelocity(0.3));
             break;

        case WB_KEYBOARD_DOWN:
             wb_motor_set_velocity(first_motor,-VELOCITY_MANUAL);
             wb_motor_set_velocity(second_motor,VELOCITY_MANUAL);
             wb_motor_set_velocity(third_motor,0);
             printf("Linear Velocity is: %.2lf\n",
             angularToLinealVelocity(0.3));
             break;

        case WB_KEYBOARD_LEFT:
             wb_motor_set_velocity(first_motor,VELOCITY_MANUAL);
             wb_motor_set_velocity(second_motor,VELOCITY_MANUAL);
             wb_motor_set_velocity(third_motor,-VELOCITY_MANUAL*1.85);
             printf("Linear Velocity is: %.2lf\n",
             angularToLinealVelocity(0.3));
             break;

        case WB_KEYBOARD_RIGHT:
             wb_motor_set_velocity(first_motor,-VELOCITY_MANUAL);
             wb_motor_set_velocity(second_motor,-VELOCITY_MANUAL);
             wb_motor_set_velocity(third_motor,VELOCITY_MANUAL*1.85);
             printf("Linear Velocity is: %.2lf\n",
             angularToLinealVelocity(0.3));
             break;

        case 'A':
             wb_motor_set_velocity(first_motor,VELOCITY_MANUAL);
             wb_motor_set_velocity(second_motor,VELOCITY_MANUAL);
             wb_motor_set_velocity(third_motor,VELOCITY_MANUAL);
             printf("Degrees/s are: %d\n", 45);
             break;

        case 'S':
             wb_motor_set_velocity(first_motor,-VELOCITY_MANUAL);
             wb_motor_set_velocity(second_motor,-VELOCITY_MANUAL);
             wb_motor_set_velocity(third_motor,-VELOCITY_MANUAL);
             printf("Degrees/s are: %d\n", 45);
             break;
        default:
             wb_motor_set_velocity(first_motor,0);
             wb_motor_set_velocity(second_motor,0);
             wb_motor_set_velocity(third_motor,0);
             printf("Linear Velocity is: %.2lf\n",
             angularToLinealVelocity(0));
             break;
    }
}

void autonomous(WbDeviceTag first_motor, WbDeviceTag second_motor,
                WbDeviceTag third_motor, double distance_sensor_value1,
                double distance_sensor_value2, float desired_centimeters,
                WbDeviceTag detector_motor, WbDeviceTag gun_motor,
                WbDeviceTag detector_sensor, WbDeviceTag gun_sensor,
                WbDeviceTag detector_distance, WbDeviceTag gun_distance){

    wb_motor_set_velocity(detector_motor, 3);


    double position_gun_value = wb_position_sensor_get_value
                                       (gun_sensor);
    double position_detector_value = wb_position_sensor_get_value
                                       (detector_sensor);
    double distance_value = wb_distance_sensor_get_value
                                  (detector_distance);
    double distance_gun = wb_distance_sensor_get_value(gun_distance);

    float distance1 = resolutionToCentimeters(35);
    float distance2 = resolutionToCentimeters(25);
    float distance3 = resolutionToCentimeters(20);

    wb_motor_set_velocity(first_motor, VELOCITY_AUTONOMOUS);
    wb_motor_set_velocity(second_motor, -VELOCITY_AUTONOMOUS);
    wb_motor_set_velocity(third_motor, 0);

    if (position_detector_value >= 2*PI) {
        new_position = rpm2Radians(position_detector_value);

    }

    if (distance_value < distance1 && flg1 == 0 && distance_value > distance2) {
        flg1 = 1;
        flg2 = 0;
        flg3 = 0;
        new_position_enemy1 = new_position;
    }

    if (flg1 == 1) {
        wb_motor_set_velocity(first_motor, 0);
        wb_motor_set_velocity(second_motor, 0);
        wb_motor_set_velocity(third_motor, 0);

        wb_motor_set_velocity(gun_motor, 3);

        wb_motor_set_position(gun_motor, new_position_enemy1);
        printf("THATHATHA\n");
    }

    if (distance_value < distance2 && flg2 == 0 && distance_value > distance3) {
        flg1 = 0;
        flg2 = 1;
        flg3 = 0;
        new_position_enemy2 = new_position;
    }

    if (flg2 == 1) {
        wb_motor_set_velocity(first_motor, 0);
        wb_motor_set_velocity(second_motor, 0);
        wb_motor_set_velocity(third_motor, 0);

        wb_motor_set_velocity(gun_motor, 3);

        wb_motor_set_position(gun_motor, new_position_enemy2);
        printf("THATHATHATHATHA\n");
    }

    if (distance_value < distance3 && flg2 == 0 && distance_value > 0) {
        flg1 = 0;
        flg2 = 0;
        flg3 = 1;
        new_position_enemy3 = new_position;
    }

    if (flg3 == 1) {
        wb_motor_set_velocity(first_motor, 0);
        wb_motor_set_velocity(second_motor, 0);
        wb_motor_set_velocity(third_motor, 0);

        wb_motor_set_velocity(gun_motor, 3);

        wb_motor_set_position(gun_motor, new_position_enemy3);
        printf("THATHATHATHATHATHATHATHA\n");
    }

    //printf("distance_value: %.4lf\n", distance_value);
    //printf("distance1: %.4lf\n", distance1);
    //printf("new_position %.4lf\n", new_position);

    if (distance_sensor_value2 <= desired_centimeters && distance_sensor_value2
        < distance_sensor_value1) {
        angle_left++;
    }
        ///TRURN LEFT//

    if (angle_left >= 1 && angle_left <= 120) {
       wb_motor_set_velocity(first_motor, degreesSec2RadSec());
       wb_motor_set_velocity(second_motor, degreesSec2RadSec());
       wb_motor_set_velocity(third_motor, degreesSec2RadSec());
       angle_left++;

    } else {
        angle_left = 0;
    }
    /* AVOID OBSTACLES RIGHT */
    if (distance_sensor_value1 < desired_centimeters && distance_sensor_value1
        < distance_sensor_value2) {
        angle_right++;
    }
    if (angle_right >= 1 && angle_right <= 120) {
        wb_motor_set_velocity(first_motor, -degreesSec2RadSec());
        wb_motor_set_velocity(second_motor, -degreesSec2RadSec());
        wb_motor_set_velocity(third_motor, -degreesSec2RadSec());
        angle_right++;
    } else {
        angle_right = 0;
    }

}


enum {
    AUTONOMOUS,
    MANUAL
};



int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
   wb_robot_init();

   WbDeviceTag first_motor = wb_robot_get_device("wheel1");
   WbDeviceTag second_motor = wb_robot_get_device("wheel2");
   WbDeviceTag third_motor = wb_robot_get_device("wheel3");
   WbDeviceTag detector_motor = wb_robot_get_device("detector_motor");
   WbDeviceTag gun_motor = wb_robot_get_device("gun_motor");

   WbDeviceTag right_distance = wb_robot_get_device("right_light");
   WbDeviceTag left_distance = wb_robot_get_device("left_light");
   WbDeviceTag detector_distance = wb_robot_get_device("detector_distance");
   WbDeviceTag gun_distance  = wb_robot_get_device("gun_distance");

   WbDeviceTag ps1 = wb_robot_get_device("first_ps");
   WbDeviceTag ps2 = wb_robot_get_device("second_ps");
   WbDeviceTag ps3 = wb_robot_get_device("third_ps");
   WbDeviceTag detector_sensor = wb_robot_get_device("detector_sensor");
   WbDeviceTag gun_sensor = wb_robot_get_device("gun_sensor");


   wb_motor_set_position(first_motor, INFINITY);
   wb_motor_set_position(second_motor, INFINITY);
   wb_motor_set_position(third_motor, INFINITY);
   wb_motor_set_position(detector_motor, INFINITY);

   wb_distance_sensor_enable(right_distance, TIME_STEP);
   wb_distance_sensor_enable(left_distance, TIME_STEP);
   wb_distance_sensor_enable(detector_distance, TIME_STEP);
   wb_distance_sensor_enable(gun_distance, TIME_STEP);

   wb_position_sensor_enable(ps1, TIME_STEP);
   wb_position_sensor_enable(ps2, TIME_STEP);
   wb_position_sensor_enable(ps3, TIME_STEP);
   wb_position_sensor_enable(detector_sensor, TIME_STEP);
   wb_position_sensor_enable(gun_sensor, TIME_STEP);

   wb_keyboard_enable(TIME_STEP);

   double distance_sensor_value1;
   double distance_sensor_value2;

   double position_sensor_value1;
   double position_sensor_value2;
   double position_sensor_value3;

   float desired_centimeters = resolutionToCentimeters(DISTANCE_OBSTACLE);

   int key;
   int robot_status=0;



  while (wb_robot_step(TIME_STEP) != -1) {

      key = wb_keyboard_get_key();

      if (key == 'W') {
          robot_status = MANUAL;
      } else if (key == 'G') {
          robot_status = AUTONOMOUS;
      } else {
          stopMotors(first_motor, second_motor, third_motor);
      }

      distance_sensor_value1 = wb_distance_sensor_get_value(right_distance);
      distance_sensor_value2 = wb_distance_sensor_get_value(left_distance);

      switch (robot_status) {
          case MANUAL:
          manual(key, first_motor, second_motor, third_motor);
          break;
          case AUTONOMOUS:
          autonomous(first_motor, second_motor, third_motor,
          distance_sensor_value1, distance_sensor_value2,
          desired_centimeters, detector_motor, gun_motor, detector_sensor,
          gun_sensor, detector_distance, gun_distance);
          break;
      }
      // current_time = wb_robot_get_time();
      distance_sensor_value1 = wb_distance_sensor_get_value(right_distance);
      distance_sensor_value2 = wb_distance_sensor_get_value(left_distance);

      position_sensor_value1 = wb_position_sensor_get_value(ps1);
      position_sensor_value2 = wb_position_sensor_get_value(ps2);
      position_sensor_value3 = wb_position_sensor_get_value(ps3);

      // printf("%s  %.2f || %s %.2f || %s %.2f || %s %.2f || %s %.2f\n",
      // file1, distance_sensor_value1, file2, position_sensor_value1, file3,
      // distance_sensor_value2, file4, position_sensor_value2, file5,
      // position_sensor_value3);
      // printf("\t\t\t\t     %s %.2fm || %s %.2fm\n",
      // file1, distance_sensor_value1 * 0.2/MAX_BITS, file3,
      // distance_sensor_value2 * 0.2/MAX_BITS);

  };


  wb_robot_cleanup();

  return 0;
}
