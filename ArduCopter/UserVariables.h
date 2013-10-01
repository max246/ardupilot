/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

//***** SONAR TO TAKE DISTANCE FROM THE WALL! ******
static AP_HAL::AnalogSource *sonar_wall_analog_source = hal.analogin->channel(1);
float scaling = 5;
static AP_RangeFinder_MaxsonarXL *sonarWall = new AP_RangeFinder_MaxsonarXL(sonar_wall_analog_source,
            &sonar_mode_filter);

static int16_t pitch_control = 0;

static uint8_t s_stage = 0;

static int16_t min_distance_wall = 80;

static int16_t s_sonar_reading = 0;

static int16_t s_distance_wall = 40; //distance from the wall, in cm
static int16_t s_dz_wall = 50; // this is the dead zone where the drone can stay after its far away from the min distance
//*** state after the dead zone 
static int16_t s_min_far_away = s_distance_wall + s_dz_wall; //min distance after the dead zone, here the drone needs to move back to keep the position
static int16_t s_mid_far_away = s_min_far_away + (s_distance_wall/2); //the middle distance between min and max
//**** state before the dead zone
static int16_t s_max_close = s_distance_wall; //this is the max distance, if the drone goes under this, the drone has to move on the dz
static int16_t s_mid_close = s_distance_wall/2; // this is very dangerous point, the drone must never go that close to the wall, so in this case high speed back

static int16_t s_calc_delta = 0; //calculate how far is it



#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


