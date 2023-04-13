#define MIN(X, Y) ((X) > (Y) ? (Y) : (X))
#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))

#define EST_2D_ALL
#define LOCAL_GAIN 0.5
#define GLOBAL_GAIN 20

// num of leaders
#define NUM_CLUSTERS 7

// num of agents per cluster
#define CLUSTER_SIZE 5.0


#define TARGET_Y (double[]){1.0, 0.82, 0.65, 0.49, 0.34, 0.22, 0.12, 0.05, 0.01, 0.0, 0.03, 0.08, 0.17, 0.28, 0.41, 0.57, 0.73, 0.91, 1.09, 1.27, 1.43, 1.59, 1.72, 1.83, 1.92, 1.97, 2.0, 1.99, 1.95, 1.88, 1.78, 1.66, 1.51, 1.35, 1.18}
#define TARGET_X (double[]){0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8}
#define DIRECTION (double[]){0.0, 10.29, 20.57, 30.86, 41.14, 51.43, 61.71, 72.0, 82.29, 92.57, 102.86, 113.14, 123.43, 133.71, 144.0, 154.29, 164.57, 174.86, 185.14, 195.43, 205.71, 216.0, 226.29, 236.57, 246.86, 257.14, 267.43, 277.71, 288.0, 298.29, 308.57, 318.86, 329.14, 339.43, 349.71}

double get_bearing_in_rad(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[1], north[0]);
  return rad;
  // double bearing = (rad - 1.5708) / M_PI * 180.0;
  // if (bearing < 0.0)
    // bearing = bearing + 360.0;
  // return bearing;
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
  
  return_val[0] = (double) sqrt( dist_y*dist_y + dist_x*dist_x );
  return_val[1] = (double) destinationBearing - currentBearing;
}