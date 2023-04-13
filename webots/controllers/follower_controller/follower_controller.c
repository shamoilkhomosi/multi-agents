/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/gps.h>
#include <webots/compass.h>

#include <../parameters.h>

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define MAX_VELOCITY 7.536
// range 1024/2
#define RANGE (4095)
#define MAX_DS (0.5)
#define PI 3.14159

static double get_z_ideal(WbDeviceTag tof_sensor, int leader, int id, WbDeviceTag gps, WbDeviceTag compass);
static double get_z_constant(WbDeviceTag receiver, double *z, int id);
static double get_z_linear(WbDeviceTag receiver, double *z, int id);
static double get_z_all(WbDeviceTag receiver, double *z, int id);
static double get_z_2d_all(WbDeviceTag receiver, double *z, double *b, int id, WbDeviceTag gps, WbDeviceTag compass, int leader);
static void write_to_csv(int id, double distance);

int main(int argc, char *argv[]) {
  /* define variables */
  WbDeviceTag compass, gps, receiver, tof_sensor, distance_sensor[8], left_motor, right_motor;
  int i, j;
  double speed[3];
  double g_speed[2]= {0, 0}; // m/s
  double l_speed[2]; // m/s
  double speed_r[3]; double speed_l[3]; 
  int time_step;
  int camera_time_step;
  double dist_temp;
  double dist_left, dist_right;
  double disp_left, disp_right;
  int id, leader;
  char sid;
  bool collision;
  double collision_angle;
  /* initialize Webots */
  wb_robot_init();
  // wb_robot_init();
  // wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    #if defined(EST_ALL)
    time_step = 64;
    #else
    time_step = 64;
    #endif
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 256;
    camera_time_step = 1024;
  }
  
  /*  receiver settings for followers only */
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
    /* gps settings for leaders */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, time_step);
  
  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  char *name = wb_robot_get_name();

  for (i = 0; i < 16; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%x", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }
  
  // get tof sensor
  tof_sensor = wb_robot_get_device("tof");
  wb_distance_sensor_enable(tof_sensor, time_step);

  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    dist_right = MAX_DS - 0.1;
    for (i = 1; i < 8; i++){
      dist_temp = (MAX_DS - (wb_distance_sensor_get_value(distance_sensor[i]) / RANGE)*MAX_DS);
      if (fabs(dist_temp) < fabs(dist_right)){
        dist_right = dist_temp;
        disp_right = dist_right*sin( PI/2 - i*PI/8 );
        
        if (dist_right < COLLISION_MIN_DISTANCE){
          collision = true;
          collision_angle = -i*PI/8;
        }
        else{
          collision = false;
        }
      }
    }
    
    dist_left = MAX_DS - 0.1;
    for (i = 9; i < 16; i++){
      dist_temp = (MAX_DS - (wb_distance_sensor_get_value(distance_sensor[i]) / RANGE)*MAX_DS);
      if (fabs(dist_temp) < fabs(dist_left)){
        dist_left = dist_temp;
        disp_left = dist_left*sin( 1.5*PI  -  (i-8)*PI/8 );
        
        if (dist_right < COLLISION_MIN_DISTANCE){
          if (!(collision && dist_left>dist_right)){
            collision_angle = 2*PI - i*PI/8;
            collision = true;
          }
        }
        else{
          collision = false;
        }
      }
    }
    
    /* set speed values */
    int boundary = !(strcmp(strrchr(name, '\0') - 2 , "rb") && strcmp(strrchr(name, '\0') - 2 , "lb"));
    if (strcmp(strrchr(name, '\0') - 2 , "rb") == 0){
      l_speed[1] = disp_left/0.01;
      sid = name[strlen(name)-4];
    }
    else if (strcmp(strrchr(name, '\0') - 2, "lb") == 0){
      l_speed[1] = disp_right/0.01;
      sid = name[strlen(name)-4];
    }
    else{
      l_speed[1] = (disp_right+disp_left)/0.01;
      sid = name[strlen(name)-1];
    }
    
    id = (int)sid - 48; // This will not work with agent ID of more than one character!!
    leader = (int)name[7] - 48;
    
    #ifdef EST_IDEAL
    g_speed[1] = get_z_ideal(tof_sensor, leader, id, gps, compass);
    #elif defined(EST_CONSTANT)
    g_speed[1] = get_z_constant(receiver, &g_speed[1], id);
    #elif defined(EST_LINEAR) 
    g_speed[1] = get_z_linear(receiver, &g_speed[1], id);
    #elif defined(EST_QUAD) 
    g_speed[1] = get_z_linear(receiver, &g_speed[1], id);
    #elif defined(EST_ALL)
    g_speed[1] = get_z_all(receiver, &g_speed[1], id);
    #endif
    
    #if defined(EST_2D_ALL)
    double target_dist;
    double target_bearing;
    get_z_2d_all(receiver, &target_dist, &target_bearing, id, gps, compass, leader);
    
    double rbt_v = target_dist;
    if (collision) target_bearing = -(collision_angle + PI); // for collision avoidance - rotate opposite obstacle
    if (collision) printf("COLLISION!!!!!!!!\n");
    double rbt_w = -target_bearing;
    double spd_right = rbt_v - rbt_w;
    double spd_left = rbt_v + rbt_w;
     
    speed_l[0] = + LOCAL_GAIN * l_speed[1] + GLOBAL_GAIN * spd_left; // angular velocity from wheel radius
    speed_r[0] = + LOCAL_GAIN * l_speed[1] + GLOBAL_GAIN * spd_right; // angular velocity from wheel radius
    printf("%i: %lf %lf %lf\n", id, target_dist, speed_l[0], speed_r[0]);
    
    speed_l[0] = 0.33*speed_l[0] + 0.33*speed_l[1] + 0.33*speed_l[2];
    speed_r[0] = 0.33*speed_r[0] + 0.33*speed_r[1] + 0.33*speed_r[2];
    wb_motor_set_velocity(left_motor, MIN(MAX_VELOCITY, speed_l[0]));
    wb_motor_set_velocity(right_motor, MIN(MAX_VELOCITY, speed_r[0]));
    speed_l[2] = speed_l[1]; speed_l[1] = speed_l[0];
    speed_r[2] = speed_r[1]; speed_r[1] = speed_r[0];
    
    #else
    // printf("%c: %lf\n", sid, g_speed[1]);
   
    speed[0] = + LOCAL_GAIN * l_speed[1] + GLOBAL_GAIN * g_speed[1];
    // printf("%i: %lf (%lf, %lf), %lf\n", id, l_speed[1], disp_left, disp_right, g_speed[1]);
    
    /* set speed values */
    speed[0] = 0.33*speed[0] + 0.33*speed[1] + 0.33*speed[2];
    if (!boundary){
    wb_motor_set_velocity(left_motor, MIN(MAX_VELOCITY, speed[0]));
    wb_motor_set_velocity(right_motor, MIN(MAX_VELOCITY, speed[0]));
    } else{
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
    }
    speed[2] = speed[1];
    speed[1] = speed[0];
    #endif
    
    write_to_csv((leader-1)*(int)CLUSTER_SIZE + id - 1, get_z_ideal(tof_sensor, leader, id, gps, compass));
    
  }

  wb_robot_cleanup();

  return 0;
}

static double get_z_ideal(WbDeviceTag tof_sensor, int leader, int id, WbDeviceTag gps, WbDeviceTag compass){
  double return_val[2] = {};
  CalcDistanceToDestination(return_val, TARGET_Y[(leader-1)*(int)CLUSTER_SIZE + id - 1], TARGET_X[(leader-1)*(int)CLUSTER_SIZE + id - 1], gps, compass); // (add reference tracking here) all in m

  return return_val[0];
}

static double get_z_constant(WbDeviceTag receiver, double *z, int id){
  /* receiver processing */
  double temp;
  
  while (wb_receiver_get_queue_length(receiver) > 0) {
    const double *message = wb_receiver_get_data(receiver);
    
    temp = *message;
    if (round(*message*100) != 0){ // prevent noisy/faulty measurements from being used
      *z = temp; 
      // printf("%i: %lf\n", id, temp);
    }
    
    wb_receiver_next_packet(receiver);
  } 
  
  return *z;
 }

static double get_z_linear(WbDeviceTag receiver, double *z, int id){
  /* receiver processing */
  double temp;
  
  while (wb_receiver_get_queue_length(receiver) > 0) {
    const double *message = wb_receiver_get_data(receiver);
    
    if (round(*message*100) != 0){ // prevent noisy/faulty measurements from being used
      *z = *message; 
      // printf("FOLLOWER stat: %d id: %d rcvd: %lf\n", wb_receiver_get_channel(receiver), id, *message);
    }
    
    wb_receiver_next_packet(receiver); 
  } 
   
  return *z;
}

static double get_z_all(WbDeviceTag receiver, double *z, int id){
  double max_strength, gain;
  double temp;
  int size, queue;
  wb_receiver_set_channel(receiver, 1);
   
  while (wb_receiver_get_queue_length(receiver) > 0) {
    const double *message = wb_receiver_get_data(receiver);
    double signal = (wb_receiver_get_signal_strength(receiver)); // square rooted because signal => 1/(dist)^2 - try mult by a large gain to avoid strength becoming too small
    size = wb_receiver_get_data_size(receiver);
    queue = wb_receiver_get_queue_length(receiver);
    // could have secondary echoing - distant followers transmit leaders' messages to other followers
    // extremities have a hard time following - local gain for followers should be stronger?
    
    // if (signal > max_strength){
      // max_strength = signal;
    // }
    max_strength = 100; // = 1/h^2 for h = 0.1
    
    
    temp = *message;
    if (round(temp*1000) != 0){ // prevent noisy/faulty measurements from being used
      gain = signal/max_strength; // multiplied by 0.9 to avoid a gain = 1
      // bound by an upper value based on minimum distance possible -> constant Amax = 1/(r_min)... no need for gain
      *z = gain*temp + (1-gain)*(*z); 
      printf("%i: %lf (%lf) %lf ... %i\n", id, temp, signal, *z, queue);
    }
    
    wb_receiver_next_packet(receiver);
  } 
  
  return *z;
 }
 
static double get_z_2d_all(WbDeviceTag receiver, double *z, double *b, int id, WbDeviceTag gps, WbDeviceTag compass, int leader){
  // double max_strength, gain;
  // double temp;
  // int size, queue;
  // wb_receiver_set_channel(receiver, 1);
   
  // while (wb_receiver_get_queue_length(receiver) > 0) {
    // const double *message = wb_receiver_get_data(receiver);
    // double signal = (wb_receiver_get_signal_strength(receiver)); // square rooted because signal => 1/(dist)^2 - try mult by a large gain to avoid strength becoming too small
    // size = wb_receiver_get_data_size(receiver);
    // queue = wb_receiver_get_queue_length(receiver);

    // max_strength = 100; // = 1/h^2 for h = 0.1
    
    
    // temp = *message;
    // if (round(temp*1000) != 0){ // prevent noisy/faulty measurements from being used
      // gain = signal/max_strength; // multiplied by 0.9 to avoid a gain = 1
      // *z = gain*temp + (1-gain)*(*z); 
      // printf("%i: %lf (%lf) %lf ... %i\n", id, temp, signal, *z, queue);
    // }
    
    // wb_receiver_next_packet(receiver);
  // } 
  
  // the ideal case
  double return_val[2] = {};
  CalcDistanceToDestination(return_val, TARGET_Y[(leader-1)*(int)CLUSTER_SIZE + id - 1], TARGET_X[(leader-1)*(int)CLUSTER_SIZE + id - 1], gps, compass); // (add reference tracking here) all in m
  *z = return_val[0];
  *b = return_val[1];
  
  return *z;
 }
 
 static void write_to_csv(int id, double distance){
  FILE *fpt;
  if (access(FNAME, F_OK) == 0) {
    fpt = fopen(FNAME, "a");
    fprintf(fpt,"%d, %lf\n", id, distance);
    fclose(fpt);
  }
  else {
    fpt = fopen(FNAME, "w");
    fprintf(fpt,"ID, Distance\n");
    fclose(fpt);
  }
}