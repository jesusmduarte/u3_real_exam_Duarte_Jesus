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

#define RADIUS_WHEELS 0.07
#define PI 3.14
#define MAX_BITS 65535
#define SPIN_DEGREE 45
//#define angle_left 0
//#define angle_right 0

#define MAX_VELOCITY 30.36
#define VELOCITY_AUTONOMOUS -8
#define VELOCITY_MANUAL -6
#define DISTANCE_OBSTACLE 17



/*functions*/
float resolutionToCentimeters(float centimeters){
    return (MAX_BITS*20)/(MAX_BITS);
}

void stopMotors(WbDeviceTag first_motor, WbDeviceTag second_motor,
                   WbDeviceTag third_motor){
    wb_motor_set_velocity(first_motor, 0);
    wb_motor_set_velocity(second_motor, 0);
    wb_motor_set_velocity(third_motor, 0);
}

float angularToLinealVelocity(float meters_per_second){
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

void manual(int key, WbDeviceTag first_motor, WbDeviceTag second_motor,
            WbDeviceTag third_motor){
    switch (key) {
        
        case WB_KEYBOARD_UP:    wb_motor_set_velocity(first_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,0);
                                printf("Linear Velocity is: %.2lf\n",
                                angularToLinealVelocity(0.3));
                                break;
        
        case WB_KEYBOARD_DOWN:  wb_motor_set_velocity(first_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,0);
                                printf("Linear Velocity is: %.2lf\n",
                                angularToLinealVelocity(0.3));
                                break;
        
        case WB_KEYBOARD_LEFT:  wb_motor_set_velocity(first_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,-VELOCITY_MANUAL*1.85);                                
                                printf("Linear Velocity is: %.2lf\n",
                                angularToLinealVelocity(0.3));
                                break;
        
        case WB_KEYBOARD_RIGHT: wb_motor_set_velocity(first_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,VELOCITY_MANUAL*1.85);
                                printf("Linear Velocity is: %.2lf\n",
                                angularToLinealVelocity(0.3));
                                break;
        
        case 'A':               wb_motor_set_velocity(first_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,VELOCITY_MANUAL);
                                printf("Degrees/s are: %d\n", 45);
                                break;
        
        case 'S':               wb_motor_set_velocity(first_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,-VELOCITY_MANUAL);
                                printf("Degrees/s are: %d\n", 45);
                                break;
        default:                wb_motor_set_velocity(first_motor,0);
                                wb_motor_set_velocity(second_motor,0);
                                wb_motor_set_velocity(third_motor,0);
                                printf("Linear Velocity is: %.2lf\n",
                                angularToLinealVelocity(0));
                                break;
    }
}

void autonomous(WbDeviceTag first_motor, WbDeviceTag second_motor,
                WbDeviceTag third_motor, double distance_sensor_value1,
                double distance_sensor_value2, float desired_centimeters){

    int angle_left = 0;
    int angle_right = 0;
    /* MOVE FORWARD */
    if (distance_sensor_value1 > desired_centimeters || distance_sensor_value2
        > desired_centimeters) {

       wb_motor_set_velocity(first_motor, VELOCITY_AUTONOMOUS);
       wb_motor_set_velocity(second_motor, -VELOCITY_AUTONOMOUS);
       wb_motor_set_velocity(third_motor, 0);
       printf("Linear Velocity is: %.2lfm\n", angularToLinealVelocity(0.4));
    }

    if (distance_sensor_value2 <= desired_centimeters && distance_sensor_value2
        < distance_sensor_value1) {
        angle_left++;
    }

    /* STOP */
    

        ///TRURN LEFT//
    
    if (angle_left >= 1 && angle_left <= 70) {
       wb_motor_set_velocity(first_motor,- degreesSec2RadSec());
       wb_motor_set_velocity(second_motor,- degreesSec2RadSec());
       wb_motor_set_velocity(third_motor,- degreesSec2RadSec());
       printf("Degrees/s are: %ddeg/s\n", 45);
       angle_left++;
      
    }
    else {
        angle_left = 0;
    }
    /* AVOID OBSTACLES RIGHT */
    if (distance_sensor_value1 < desired_centimeters && distance_sensor_value1
        < distance_sensor_value2) {
        angle_right++;
    }
    if (angle_right >= 1 && angle_right <= 70) {
        wb_motor_set_velocity(first_motor, degreesSec2RadSec());
        wb_motor_set_velocity(second_motor, degreesSec2RadSec());
        wb_motor_set_velocity(third_motor, degreesSec2RadSec());
        printf("Degrees/s are: %ddeg/s\n", 45);
        angle_right++;
    }
    else {
        angle_right = 0;
    }

}


enum {
    AUTONOMOUS,
    MANUAL
};

/* global variables */
int angle_left, angle_right = 0;
char *file1 = "Distance sensor right value:";
char *file2 = "Position sensor motor1:";
char *file3 = "Distance sensor left value:";
char *file4 = "Position sensor motor2:";
char *file5 = "Position sensor motor3:";

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  WbDeviceTag first_motor = wb_robot_get_device("wheel1");
   WbDeviceTag second_motor = wb_robot_get_device("wheel2");
   WbDeviceTag third_motor = wb_robot_get_device("wheel3");

   WbDeviceTag right_distance = wb_robot_get_device("right_light");
   WbDeviceTag left_distance = wb_robot_get_device("left_light");

   WbDeviceTag ps1 = wb_robot_get_device("first_ps");
   WbDeviceTag ps2 = wb_robot_get_device("second_ps");
   WbDeviceTag ps3 = wb_robot_get_device("third_ps");

   wb_motor_set_position(first_motor, INFINITY);
   wb_motor_set_position(second_motor, INFINITY);
   wb_motor_set_position(third_motor, INFINITY);

   wb_distance_sensor_enable(right_distance, TIME_STEP);
   wb_distance_sensor_enable(left_distance, TIME_STEP);

   wb_position_sensor_enable(ps1, TIME_STEP);
   wb_position_sensor_enable(ps2, TIME_STEP);
   wb_position_sensor_enable(ps3, TIME_STEP);

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
      }
      else if (key == 'G') {
          robot_status = AUTONOMOUS;
      }
      else {
          stopMotors(first_motor, second_motor, third_motor);
      }

      distance_sensor_value1 = wb_distance_sensor_get_value(right_distance);
      distance_sensor_value2 = wb_distance_sensor_get_value(left_distance);

      switch (robot_status) {
          case MANUAL:     manual(key, first_motor, second_motor, third_motor);
                           break;
          case AUTONOMOUS: autonomous(first_motor, second_motor, third_motor,
                           distance_sensor_value1, distance_sensor_value2,
                           desired_centimeters);
                           break;
      }
      // current_time = wb_robot_get_time();
      distance_sensor_value1 = wb_distance_sensor_get_value(right_distance);
      distance_sensor_value2 = wb_distance_sensor_get_value(left_distance);

      position_sensor_value1 = wb_position_sensor_get_value(ps1);
      position_sensor_value2 = wb_position_sensor_get_value(ps2);
      position_sensor_value3 = wb_position_sensor_get_value(ps3);

      printf("%s  %.2f || %s %.2f || %s %.2f || %s %.2f || %s %.2f\n",
      file1, distance_sensor_value1, file2, position_sensor_value1, file3,
      distance_sensor_value2, file4, position_sensor_value2, file5,
      position_sensor_value3);
      printf("\t\t\t\t     %s %.2fm || %s %.2fm\n",
      file1, distance_sensor_value1 * 0.2/MAX_BITS, file3,
      distance_sensor_value2 * 0.2/MAX_BITS);

  };


  wb_robot_cleanup();

  return 0;
}
