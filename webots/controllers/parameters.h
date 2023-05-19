#define MIN(X, Y) ((X) > (Y) ? (Y) : (X))
#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))

#define EST_QUAD
#define LOCAL_GAIN 1
#define GLOBAL_GAIN 20

// num of leaders
#define NUM_CLUSTERS 5

// num of agents per cluster
#define CLUSTER_SIZE 8.0
#define COLLISION_MIN_DISTANCE 0.015
const char* FNAME = "/Users/shamoilkhomosi/Desktop/Sheffield/FYP/simulation/webots/worlds/sim_data_EST_QUAD.csv";
#define TARGET_Y (double[]){0.0, 0.25649152772034317, 0.5063487333087199, 0.7431088957403889, 0.9606480576295104, 1.1533394254163996, 1.3161989100852927, 1.4450140439061934, 1.5364529386844472, 1.5881504672412288, 1.598769438984238, 1.5680351872222218, 1.49674267359967, 1.3867359258908074, 1.240860341007871, 1.0628890869377485, 0.8574255072693119, 0.6297840526825601, 0.3858528191849554, 0.13194124863391152, -0.1253830691283598, -0.37946427130749605, -0.6237303804276104, -0.8518632928529677, -1.0579622006957412, -1.2366962200946652, -1.3834422780337503, -1.4944046911677558, -1.5667133436694067, -1.598497924663769, -1.588936305049657, -1.5382758024097882, -1.4478267839787748, -1.3199287731323976, -1.157889936076122, -0.9659015139494713, -0.7489294136161786, -0.5125857612156708, -0.262983740827771, -0.0065794729441060315}
#define TARGET_X (double[]){0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9}
#define DIRECTION (double[]){0.0, 9.0, 18.0, 27.0, 36.0, 45.0, 54.0, 63.0, 72.0, 81.0, 90.0, 99.0, 108.0, 117.0, 126.0, 135.0, 144.0, 153.0, 162.0, 171.0, 180.0, 189.0, 198.0, 207.0, 216.0, 225.0, 234.0, 243.0, 252.0, 261.0, 270.0, 279.0, 288.0, 297.0, 306.0, 315.0, 324.0, 333.0, 342.0, 351.0}

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
  
  #if defined(EST_2D_ALL)
  return_val[0] = (double) sqrt( dist_y*dist_y + dist_x*dist_x );
  #else
  return_val[0] = dist_y;
  #endif
  return_val[1] = (double) destinationBearing - currentBearing;
}
