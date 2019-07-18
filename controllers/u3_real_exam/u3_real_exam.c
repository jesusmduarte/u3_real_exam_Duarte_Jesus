/*
 * File:          u3_project_Duarte_Jesus.c
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

#define MAX_VELOCITY 30.36
#define VELOCITY_AUTONOMOUS -8
#define VELOCITY_MANUAL -6
#define DISTANCE_OBSTACLE 17

/*functions*/
float bitsToCentimeters(float centimeters){
    return (MAX_BITS*20)/(MAX_BITS);
}

void stopAllWheels(WbDeviceTag first_motor, WbDeviceTag second_motor,
                   WbDeviceTag third_motor){
    wb_motor_set_velocity(first_motor, 0);
    wb_motor_set_velocity(second_motor, 0);
    wb_motor_set_velocity(third_motor, 0);
}

float linearVelocity(float meters_per_second){
    float RPM;
    float linear_velocity;

    meters_per_second = meters_per_second / RADIUS_WHEELS;
    RPM = (meters_per_second * 290) / MAX_VELOCITY;
    linear_velocity = ((2 * PI * RADIUS_WHEELS) / 60) * RPM;

    return linear_velocity;
}

void manual(int key, WbDeviceTag first_motor, WbDeviceTag second_motor,
            WbDeviceTag third_motor){
    switch (key) {
         /* MOVE FORWARD */
        case WB_KEYBOARD_UP:    wb_motor_set_velocity(first_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,0);
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0.3));
                                break;
        /* MOVE BACKWARD */
        case WB_KEYBOARD_DOWN:  wb_motor_set_velocity(first_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,0);
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0.3));
                                break;
        /* MOVE TO THE LEFT */
        case WB_KEYBOARD_LEFT:  wb_motor_set_velocity(first_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,-VELOCITY_MANUAL*1.85);                                
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0.3));
                                break;
        /* MOVE TO THE RIGHT */
        case WB_KEYBOARD_RIGHT: wb_motor_set_velocity(first_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,VELOCITY_MANUAL*1.85);
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0.3));
                                break;
        /* TURN TO THE LEFT */
        case 'A':               wb_motor_set_velocity(first_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,VELOCITY_MANUAL);
                                printf("Degrees/s are: %d\n", 45);
                                break;
        /* TURN TO THE RIGHT */
        case 'S':               wb_motor_set_velocity(first_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(second_motor,-VELOCITY_MANUAL);
                                wb_motor_set_velocity(third_motor,-VELOCITY_MANUAL);
                                printf("Degrees/s are: %d\n", 45);
                                break;
        default:                wb_motor_set_velocity(first_motor,0);
                                wb_motor_set_velocity(second_motor,0);
                                wb_motor_set_velocity(third_motor,0);
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0));
                                break;
    }
}

void autonomous(WbDeviceTag first_motor, WbDeviceTag second_motor,
                WbDeviceTag third_motor, double distance_sensor_value1,
                double distance_sensor_value2, float desired_centimeters){

    int flag_left = 0;
    int flag_right = 0;

    /* MOVE FORWARD */
    if ((distance_sensor_value1 > desired_centimeters) && (distance_sensor_value2 > desired_centimeters)) {

       wb_motor_set_velocity(first_motor, VELOCITY_AUTONOMOUS);
       wb_motor_set_velocity(second_motor, -VELOCITY_AUTONOMOUS);
       wb_motor_set_velocity(third_motor, 0);
    }

    /* STOP */
    else if ((distance_sensor_value1 <= desired_centimeters) && (flag_right == 0)) {
        wb_motor_set_velocity(first_motor, 0);
        wb_motor_set_velocity(second_motor, 0);
        wb_motor_set_velocity(third_motor, 0);

        flag_right = 1;
    }

    else if ((distance_sensor_value2 <= desired_centimeters) && (flag_left == 0)) {
       wb_motor_set_velocity(first_motor, 0);
       wb_motor_set_velocity(second_motor, 0);
       wb_motor_set_velocity(third_motor, 0);

       flag_left = 1;
    }


    /* AVOID OBSTACLES LEFT */
    //float angle=
    if (distance_sensor_value2 <= desired_centimeters && flag_left == 1) {
       wb_motor_set_velocity(first_motor, VELOCITY_AUTONOMOUS);
       wb_motor_set_velocity(second_motor, VELOCITY_AUTONOMOUS);
       wb_motor_set_velocity(third_motor, VELOCITY_AUTONOMOUS);

       flag_left = 0;
    }
    /* AVOID OBSTACLES RIGHT */
    if (distance_sensor_value1 <= desired_centimeters && flag_right == 1) {
        wb_motor_set_velocity(first_motor, -VELOCITY_AUTONOMOUS);
        wb_motor_set_velocity(second_motor, -VELOCITY_AUTONOMOUS);
        wb_motor_set_velocity(third_motor, -VELOCITY_AUTONOMOUS);

        flag_right = 0;
    }

}

enum {
    AUTONOMOUS,
    MANUAL
};

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
   wb_robot_init();

   WbDeviceTag first_motor = wb_robot_get_device("wheel1");
   WbDeviceTag second_motor = wb_robot_get_device("wheel2");
   WbDeviceTag third_motor = wb_robot_get_device("wheel3");
   
   WbDeviceTag fourth_motor = wb_robot_get_device("motor_4");
   WbDeviceTag five_motor = wb_robot_get_device("motor_5");
   

   WbDeviceTag right_distance = wb_robot_get_device("right_light");
   WbDeviceTag left_distance = wb_robot_get_device("left_light");

   WbDeviceTag ps1 = wb_robot_get_device("first_ps");
   WbDeviceTag ps2 = wb_robot_get_device("second_ps");
   WbDeviceTag ps3 = wb_robot_get_device("third_ps");

   wb_motor_set_position(first_motor, INFINITY);
   wb_motor_set_position(second_motor, INFINITY);
   wb_motor_set_position(third_motor, INFINITY);
   wb_motor_set_position(fourth_motor, INFINITY);
   wb_motor_set_position(five_motor, INFINITY);

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

   float desired_centimeters = bitsToCentimeters(DISTANCE_OBSTACLE);

   int key;
   int robot_status=0;
   double current_time;


  while (wb_robot_step(TIME_STEP) != -1) {

      key = wb_keyboard_get_key();

      if (key == 'W') {
          robot_status = MANUAL;
          wb_motor_set_velocity(fourth_motor, 2);
          wb_motor_set_velocity(five_motor, 1);
      }
      else if (key == 'G') {
          robot_status = AUTONOMOUS;
      }
      else {
          stopAllWheels(first_motor, second_motor, third_motor);
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
      printf("Current time: %.4f\n", current_time);
      printf("Desired centimeters %.4f\n", desired_centimeters);

      distance_sensor_value1 = wb_distance_sensor_get_value(right_distance);
      distance_sensor_value2 = wb_distance_sensor_get_value(left_distance);

      printf("Distance sensor right value is: %.4f\n", distance_sensor_value1);
      printf("Distance sensor left value is: %.4f\n", distance_sensor_value2);

      position_sensor_value1 = wb_position_sensor_get_value(ps1);
      position_sensor_value2 = wb_position_sensor_get_value(ps2);
      position_sensor_value3 = wb_position_sensor_get_value(ps3);

      printf("Position sensor wheel 1 is: %.4f\n", position_sensor_value1);
      printf("Position sensor wheel 2 is: %.4f\n", position_sensor_value2);
      printf("Position sensor wheel 3 is: %.4f\n", position_sensor_value3);

  };


  wb_robot_cleanup();

  return 0;
}
