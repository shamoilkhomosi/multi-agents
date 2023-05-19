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
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <unistd.h>

#include <../parameters.h>

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define MAX_VELOCITY 7.536

#define RANGE (4095)
#define MAX_DS (0.5)
#define PI 3.14159
#define FRAME_SIZE 128
int global_send_index = 1;

static void send_z_ideal(void);
static void send_z_constant(WbDeviceTag emitter, char *message, double z, int original_channel);
static void send_z_linear(WbDeviceTag emitter, WbDeviceTag receiver, double z, int original_channel);
static void send_z_quad(WbDeviceTag emitter, WbDeviceTag receiver, double z, int original_channel);
static void send_z_all(WbDeviceTag emitter, char *message, double z, int id);
static void send_z_2d_all(WbDeviceTag emitter, char *message, double z, double b, int id);
static void write_to_csv(int id, double distance);

int main(int argc, char *argv[]) {
  /* define variables */
  WbDeviceTag compass, gps, emitter, global_receiver, tof_sensor, distance_sensor[8], left_motor, right_motor;
  int i;
  double speed[3]; // rad/s
  double speed_r[3]; double speed_l[3]; 
  double g_speed[2]; // m/s
  double l_speed[2]; // m/s
  double tof_distance, target_dist;
  int time_step;
  int camera_time_step;
  double dist_temp;
  double dist_left, dist_right;
  double disp_left, disp_right;
  int id;
  bool collision;
  double collision_angle;
  srand(time(NULL)); 
  /* initialize Webots */
  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    // printf("e-puck2 robot\n");
    #if defined(EST_ALL)
    time_step = 64;
    #else
    time_step = 64;
    #endif
  } else {  // original e-puck
    // printf("e-puck robot\n");
    time_step = 256;
    camera_time_step = 1024;
  }
  
  /* emitter settings for leader only */
  emitter = wb_robot_get_device("emitter");
  char message[FRAME_SIZE];
  
  /* gps settings for leaders */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);
  /* compass settings for leaders */
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, time_step);
  
  /*  receiver settings for followers only */
  global_receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(global_receiver, time_step);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);


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
      if (fabs(dist_temp) < dist_right){
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
      if (fabs(dist_temp) < dist_left){
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
    
    id = wb_receiver_get_channel(global_receiver);
    
    /* set speed values */
    double return_val[2] = {0};
    CalcDistanceToDestination(return_val, TARGET_Y[(id-1)*(int)CLUSTER_SIZE+(int)floor(CLUSTER_SIZE/2)], TARGET_X[(id-1)*(int)CLUSTER_SIZE+(int)floor(CLUSTER_SIZE/2)], gps, compass);;

    target_dist = return_val[0];
    tof_distance = wb_distance_sensor_get_value(tof_sensor)/1000; // in m
    
    g_speed[1] = target_dist;
    l_speed[1] = (disp_right+disp_left)/0.01;
    
    #ifdef EST_IDEAL
    send_z_ideal();
    #elif defined(EST_CONSTANT)
    send_z_constant(emitter, message, target_dist, id);
    #elif defined(EST_LINEAR)
    send_z_linear(emitter, global_receiver, target_dist, id);
    #elif defined(EST_QUAD)
    send_z_quad(emitter, global_receiver, target_dist, id);
    #elif defined(EST_ALL)
    send_z_all(emitter, message, target_dist, id);
    #endif
    
    #if defined(EST_2D_ALL)
    
    double target_bearing = return_val[1];
    double rbt_v = target_dist;
    if (collision) target_bearing = -(collision_angle + PI); // for collision avoidance - rotate opposite obstacle
    double rbt_w = -target_bearing;
    
    double spd_right = rbt_v - rbt_w;
    double spd_left = rbt_v + rbt_w;
    
    send_z_2d_all(emitter, message, target_dist, target_bearing, id);
 
    speed_l[0] = + LOCAL_GAIN * l_speed[1] + GLOBAL_GAIN * spd_left; // angular velocity from wheel radius
    speed_r[0] = + LOCAL_GAIN * l_speed[1] + GLOBAL_GAIN * spd_right; // angular velocity from wheel radius
    // printf("leader %i: %lf %lf %lf\n", id, target_dist, speed_l[0], speed_r[0]);
    
    speed_l[0] = 0.33*speed_l[0] + 0.33*speed_l[1] + 0.33*speed_l[2];
    speed_r[0] = 0.33*speed_r[0] + 0.33*speed_r[1] + 0.33*speed_r[2];
    wb_motor_set_velocity(left_motor, MIN(MAX_VELOCITY, speed_l[0]));
    wb_motor_set_velocity(right_motor, MIN(MAX_VELOCITY, speed_r[0]));
    speed_l[2] = speed_l[1]; speed_l[1] = speed_l[0];
    speed_r[2] = speed_r[1]; speed_r[1] = speed_r[0];
    
    #else
    
    speed[0] = + LOCAL_GAIN * l_speed[1] + GLOBAL_GAIN * g_speed[1]; // angular velocity from wheel radius
    printf("leader %i: %lf %lf %lf %lf\n", id, target_dist, speed[0], l_speed[1], g_speed[1]);
    
    speed[0] = 0.33*speed[0] + 0.33*speed[1] + 0.33*speed[2];
    wb_motor_set_velocity(left_motor, MIN(MAX_VELOCITY, speed[0]));
    wb_motor_set_velocity(right_motor, MIN(MAX_VELOCITY, speed[0]));
    speed[2] = speed[1];
    speed[1] = speed[0];
    
    #endif
    
    write_to_csv((id-1)*(int)CLUSTER_SIZE+(int)floor(CLUSTER_SIZE/2), target_dist);
  }

  wb_robot_cleanup();

  return 0;
}

static void send_z_ideal(void){}

static void send_z_constant(WbDeviceTag emitter, char *message, double z, int original_channel){
  int new_channel = original_channel*100;
  for (int i = 1; i <= CLUSTER_SIZE; i++){
    new_channel++;
    sprintf(message, "%lf", z);
    wb_emitter_set_channel(emitter, new_channel);
    wb_emitter_send(emitter, &z, sizeof(double));
    wb_emitter_set_channel(emitter, original_channel);
  }
  
}

static void send_z_linear(WbDeviceTag emitter, WbDeviceTag receiver, double z, int original_channel){
  int new_channel, j, rcv_channel;
  double z_lead = z;
  double z_lead_r = z; // initialise to z so that if noisy z is received we still have a non-zero closest estimate
  double z_lead_l = z; 
  
  // send position to neighbouring leaders
  double z_send[2] = {(double)original_channel, z_lead};
  new_channel = original_channel-1;
  wb_emitter_set_channel(emitter, new_channel);
  wb_emitter_send(emitter, &z, sizeof(double));
  if (original_channel == NUM_CLUSTERS-1){ // send to right most agent so it doesnt miss out!
  wb_emitter_set_channel(emitter, original_channel+1);
  wb_emitter_send(emitter, &z, sizeof(double));
  }
  
  // receive position of neighbouring leaders
  rcv_channel = original_channel+1;
  while (wb_receiver_get_queue_length(receiver) > 0) {
    const double *z_rcvd = wb_receiver_get_data(receiver);
      if (round(*z_rcvd*100) != 0){  // prevent noisy/faulty measurements from being used
        z_lead_r = *z_rcvd; 
        // printf("INNNN\n");
        }
    wb_receiver_next_packet(receiver);
  } 
  
  // // printf("LEADER id: %d z-1: %lf z: %lf z+1: %lf\n", original_channel, z_lead_l, z_lead, z_lead_r);
  
  /* send commands to followers */
  double x_lead = floor(CLUSTER_SIZE/2);
  double x_lead_r = x_lead - CLUSTER_SIZE;
  double z_foh[(int)CLUSTER_SIZE] = {};
  double z_out;
  // for (int i = 0; i < ceil(CLUSTER_SIZE/2)-1; i++){
    // z_foh[i] = z_lead_l + (z_lead - z_lead_l)/(CLUSTER_SIZE)*(i+ceil(CLUSTER_SIZE/2));
    // j = i+1;
    // z_out = z_foh[i];
    // new_channel = original_channel*100+j;
    // // printf("LEADER%i ind: %d val: %lf val_l: %lf out: %lf\n", original_channel, i, z_lead, z_lead_l, z_out);
    // wb_emitter_set_channel(emitter, new_channel);
    // wb_emitter_send(emitter, &z_out, sizeof(double));
  // }
  // for (int i = ceil(CLUSTER_SIZE/2); i < CLUSTER_SIZE; i++){
    // z_foh[i] = z_lead + (z_lead_r - z_lead)/(CLUSTER_SIZE)*(i-floor(CLUSTER_SIZE/2));
    // j = i+1;
    // z_out = z_foh[i];
    // new_channel = original_channel*100+j;
    // // printf("LEADER stat: %d ind: %d val: %lf\n", new_channel, i, z_out);
    // // printf("LEADER%i ind: %d val: %lf val_r: %lf out: %lf\n", original_channel, i, z_lead, z_lead_r, z_out);
    // wb_emitter_set_channel(emitter, new_channel);
    // wb_emitter_send(emitter, &z_out, sizeof(double));
  // }
  
  for (int i = 0; i < CLUSTER_SIZE; i++){
    j=i;
    if (original_channel != NUM_CLUSTERS){
    z_foh[i] = z_lead + (z_lead_r-z_lead)/(CLUSTER_SIZE)*(i-CLUSTER_SIZE/2);
    } else{
    z_foh[i] = z_lead_r - (z_lead_r-z_lead)/(CLUSTER_SIZE)*(i+CLUSTER_SIZE/2);
    }
    // z_soh[i] = z_lead + (z_lead_r - z_lead)/(CLUSTER_SIZE)*(i-floor(CLUSTER_SIZE/2));
    j = i+1;
    z_out = z_foh[i];
    new_channel = original_channel*100+j;
    // printf("LEADER%i ind: %d val: %lf val_r: %lf out: %lf\n", original_channel, i, z_lead, z_lead_r, z_out);
    wb_emitter_set_channel(emitter, new_channel);
    wb_emitter_send(emitter, &z_out, sizeof(double));
    
  }
  
  wb_emitter_set_channel(emitter, original_channel);
}

static void send_z_quad(WbDeviceTag emitter, WbDeviceTag receiver, double z, int original_channel){
  int new_channel, j;
  double z_lead = z;
  double z_lead_r = z; // initialise to z so that if noisy z is received we still have a non-zero closest estimate
  double z_lead_l = z; 
  double z_lead_rr = z; // initialise to z so that if noisy z is received we still have a non-zero closest estimate
  double z_lead_ll = z; 
  double rcv_channel, rcv_z;
  int rcv_ok = 0;
  
  // send position to neighbouring leaders
  double z_send = original_channel*100 + z_lead;
  new_channel = original_channel+1;
  wb_emitter_set_channel(emitter, new_channel);
  wb_emitter_send(emitter, &z_send, sizeof(double));
  
  new_channel = original_channel-1;
  wb_emitter_set_channel(emitter, new_channel);
  wb_emitter_send(emitter, &z_send, sizeof(double));

  
  
  // receive position of neighbouring leaders
  // && (rcv_ok!=1)
  while ((wb_receiver_get_queue_length(receiver) > 0) ) {
    const double *z_rcvd = wb_receiver_get_data(receiver);
    rcv_channel = floor(*z_rcvd/100);
    rcv_z = *z_rcvd - rcv_channel*100;
    if ((int)rcv_channel == (original_channel+1)) {
      if (round(rcv_z*1000) != 0){ // prevent noisy/faulty measurements from being used
        z_lead_r = rcv_z; 
        rcv_ok = 1; // printf("INNNN RIGHT\n");
        } else {rcv_ok = 0;}
    }
    if ((int)rcv_channel == (original_channel-1)) {
      if (round(rcv_z*1000) != 0){ // prevent noisy/faulty measurements from being used
        z_lead_l = rcv_z; 
        rcv_ok = 1; // printf("INNNN LEFT\n");
        } else {rcv_ok = 0;}
    }
    wb_receiver_next_packet(receiver);
  } 
  
  // // printf("LEADER id: %d z-1: %lf z: %lf z+1: %lf\n", original_channel, z_lead_l, z_lead, z_lead_r);
  
  /* send commands to followers */
  double z_soh[(int)CLUSTER_SIZE] = {};
  double L0;double L1;double L2;
  double x_lead;double x_lead_l;double x_lead_ll;double x_lead_r;double x_lead_rr;
  x_lead = floor(CLUSTER_SIZE/2);
  x_lead_l = x_lead - CLUSTER_SIZE;
  x_lead_ll = x_lead_l - CLUSTER_SIZE;
  x_lead_r = x_lead + CLUSTER_SIZE;
  x_lead_rr = x_lead_r + CLUSTER_SIZE;
  double z_out;
  // for (int i = 0; i < ceil(CLUSTER_SIZE/2)-1; i++){
    // L0 = ((i-x_lead_l)*(i-x_lead_ll))/((x_lead-x_lead_l)*(x_lead-x_lead_ll))*z_lead;
    // L1 = ((i-x_lead)*(i-x_lead_ll))/((x_lead_l-x_lead)*(x_lead_l-x_lead_ll))*z_lead_l;
    // L2 = ((i-x_lead)*(i-x_lead_l))/((x_lead_ll-x_lead)*(x_lead_ll-x_lead_l))*z_lead_ll;
    // z_soh[i] = L0+L1+L2;
    // j = i+1;
    // z_out = z_soh[i];
    // new_channel = original_channel*100+j;
    // // printf("LEADER%i ind: %d val: %lf val_l: %lf out: %lf\n", original_channel, i, z_lead, z_lead_l, z_out);
    // wb_emitter_set_channel(emitter, new_channel);
    // wb_emitter_send(emitter, &z_out, sizeof(double));
  // }
  // for (int i = ceil(CLUSTER_SIZE/2); i < CLUSTER_SIZE; i++){
    // j=i;
    // L0 = ((j-x_lead_r)*(j-x_lead_rr))/((x_lead-x_lead_r)*(x_lead-x_lead_rr))*z_lead;
    // L1 = ((j-x_lead)*(j-x_lead_rr))/((x_lead_r-x_lead)*(x_lead_r-x_lead_rr))*z_lead_r;
    // L2 = ((j-x_lead)*(j-x_lead_r))/((x_lead_rr-x_lead)*(x_lead_rr-x_lead_r))*z_lead_rr;
    // z_soh[i] = z_lead + (z_lead_r - z_lead)/(CLUSTER_SIZE)*(i-floor(CLUSTER_SIZE/2));
    // j = i+1;
    // z_out = z_soh[i];
    // new_channel = original_channel*100+j;
    // // printf("LEADER%i ind: %d val: %lf val_r: %lf out: %lf\n", original_channel, i, z_lead, z_lead_r, z_out);
    // wb_emitter_set_channel(emitter, new_channel);
    // wb_emitter_send(emitter, &z_out, sizeof(double));
  // }
  
  // for (int i = 0; i < CLUSTER_SIZE; i++){
    // j=i;
    // L0 = ((j-x_lead)*(j-x_lead_l))/((x_lead_r-x_lead)*(x_lead_r-x_lead_l))*z_lead_r;
    // L1 = ((j-x_lead_r)*(j-x_lead_l))/((x_lead-x_lead_r)*(x_lead-x_lead_l))*z_lead;
    // L2 = ((j-x_lead_r)*(j-x_lead))/((x_lead_l-x_lead_r)*(x_lead_l-x_lead))*z_lead_l;
    // z_soh[i] = L0+L1+L2;
    // if (abs(z_soh[i]) > 1.5){
     // z_soh[i] = z_lead;
    // }
    //////z_soh[i] = z_lead + (z_lead_r - z_lead)/(CLUSTER_SIZE)*(i-floor(CLUSTER_SIZE/2));
    // j = i+1;
    // z_out = z_soh[i];
    // new_channel = original_channel*100+j;
    // // printf("LEADER stat: %d ind: %d val: %lf\n", new_channel, i, z_out);
    // // printf("LEADER%i ind: %d val: %lf val_r: %lf out: %lf\n", original_channel, i, z_lead, z_lead_r, z_out);
    // wb_emitter_set_channel(emitter, new_channel);
    // wb_emitter_send(emitter, &z_out, sizeof(double));
  // }
  
  
  for (int i = 0; i < ceil(CLUSTER_SIZE/2)-1; i++){
    if (z_lead != z_lead_l){
      z_soh[i] = z_lead_l + (z_lead - z_lead_l)/(CLUSTER_SIZE)*(i+ceil(CLUSTER_SIZE/2));
      z_out = z_soh[i];
    } else{
    z_soh[i] = z_lead;
      z_out = z_soh[i];
      // z_out = z_lead;
    }
    j = i+1;
    new_channel = original_channel*100+j;
    // printf("LEADER%i ind: %d val: %lf val_l: %lf out: %lf\n", original_channel, i, z_lead, z_lead_l, z_out);
    wb_emitter_set_channel(emitter, new_channel);
    wb_emitter_send(emitter, &z_out, sizeof(double));
  }
  for (int i = ceil(CLUSTER_SIZE/2); i < CLUSTER_SIZE; i++){
    if (z_lead != z_lead_r){
      z_soh[i] = z_lead + (z_lead_r - z_lead)/(CLUSTER_SIZE)*(i-floor(CLUSTER_SIZE/2));
      z_out = z_soh[i];
    } else if (original_channel == 3){
    z_soh[i] = z_lead + (z_lead_r - z_lead)/(CLUSTER_SIZE)*(i-floor(CLUSTER_SIZE/2));
      z_out = z_soh[i];
      printf("LE ME\n");
    } else{
    z_soh[i] = z_lead;
      z_out = z_soh[i];
      // z_out = z_lead;
    }
    j = i+1;
    new_channel = original_channel*100+j;
    // printf("LEADER%i ind: %d val: %lf val_r: %lf out: %lf\n", original_channel, i, z_lead, z_lead_r, z_out);
    wb_emitter_set_channel(emitter, new_channel);
    wb_emitter_send(emitter, &z_out, sizeof(double));
  }
  
  
  wb_emitter_set_channel(emitter, original_channel);
}

static void send_z_all(WbDeviceTag emitter, char *message, double z, int id){
  // double range = ((rand())%100 < 50) ? 0.6 : 0.6;
  // // printf( "rand range: %lf \n", range); 
  // wb_emitter_set_range(emitter, range);
  
  // if (global_send_index == id){
  sprintf(message, "%lf", z);
  wb_emitter_set_channel(emitter, 1);
  wb_emitter_send(emitter, &z, sizeof(double));
  // }
  
  // global_send_index++;
  // if (global_send_index == NUM_CLUSTERS+1) global_send_index = 1;
}

static void send_z_2d_all(WbDeviceTag emitter, char *message, double z, double b, int id){
  // double range = ((rand())%100 < 50) ? 0.6 : 0.6;
  // // printf( "rand range: %lf \n", range); 
  // wb_emitter_set_range(emitter, range);
  
  // if (global_send_index == id){
  // char message_1[64] = {};
  // char message_2[64] = {};
  // s// printf(message_1, "%lf", z);
  // s// printf(message_2, "%lf", b);
  // message = strcat(message_1, message_2);
  // wb_emitter_set_channel(emitter, 1);
  // wb_emitter_send(emitter, message, sizeof(message));
  // }
  
  // global_send_index++;
  // if (global_send_index == NUM_CLUSTERS+1) global_send_index = 1;
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