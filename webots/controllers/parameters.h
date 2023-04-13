#define MIN(X, Y) ((X) > (Y) ? (Y) : (X))
#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))

#define EST_ALL
#define LOCAL_GAIN 2
#define GLOBAL_GAIN 20

// num of leaders
#define NUM_CLUSTERS 2

// num of agents per cluster
#define CLUSTER_SIZE 25.0
#define COLLISION_MIN_DISTANCE 0.01
const char* FNAME = "/Users/shamoilkhomosi/Desktop/Sheffield/FYP/simulation/epuck/worlds/sim_data_EST_ALL.csv";
#define TARGET_Y (double[]){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TARGET_X (double[]){0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8, 4.9}
#define DIRECTION (double[]){0.0, 7.2, 14.4, 21.6, 28.8, 36.0, 43.2, 50.4, 57.6, 64.8, 72.0, 79.2, 86.4, 93.6, 100.8, 108.0, 115.2, 122.4, 129.6, 136.8, 144.0, 151.2, 158.4, 165.6, 172.8, 180.0, 187.2, 194.4, 201.6, 208.8, 216.0, 223.2, 230.4, 237.6, 244.8, 252.0, 259.2, 266.4, 273.6, 280.8, 288.0, 295.2, 302.4, 309.6, 316.8, 324.0, 331.2, 338.4, 345.6, 352.8}

double get_bearing_in_rad(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[2], north[0]);
  return rad;
}
void CalcDistanceToDestination(double* return_val, const double destinationCoordinate_y, const double destinationCoordinate_x, WbDeviceTag gps, WbDeviceTag compass) {
  // double currentCoordinate = cartesianConvertVec3fToCartesianVec2f(wb_gps_get_values(gps));
  const double *gpsRead = wb_gps_get_values(gps);
  const double currentCoordinate_y = -gpsRead[2];
  const double currentCoordinate_x = gpsRead[0];
  
  const double dist_y = -currentCoordinate_y+destinationCoordinate_y;
  const double dist_x = -currentCoordinate_x+destinationCoordinate_x;
  
  const double currentBearing = get_bearing_in_rad(compass);
  const double destinationBearing = atan2(dist_y, dist_x)  - 1.5708;
  
  printf("%lf %lf %lf %lf %lf %lf\n", currentCoordinate_y, currentCoordinate_x, currentBearing, destinationCoordinate_y, destinationCoordinate_x, destinationBearing);  
  #if defined(EST_2D_ALL)
  return_val[0] = (double) sqrt( dist_y*dist_y + dist_x*dist_x );
  #else
  return_val[0] = dist_y;
  #endif
  return_val[1] = (double) destinationBearing - currentBearing;
}
