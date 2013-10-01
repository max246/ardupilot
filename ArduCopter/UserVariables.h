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

static int16_t graffiti_control = 0;
static int16_t s_sonar_reading = 0;
static int16_t s_distance_wall = 40; //distance from the wall, in cm
static int32_t graffiti_distance_target = 75;   // Target distance to the wall, in cm.
static int32_t graffiti_distance_error = 0;     // Difference between target distance and measured distance, in cm.



#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


