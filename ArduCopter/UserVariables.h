/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

//***** SONAR TO TAKE DISTANCE FROM THE WALL! ******
static AP_HAL::AnalogSource *sonar_wall_analog_source = hal.analogin->channel(1);
float scaling = 5;
ModeFilterInt16_Size7 front_sonar_mode_filter(2);               // maximum filter size, keep drawing from bottom of sample to reject peaks.
static AP_RangeFinder_MaxsonarXL *FrontSonar = new AP_RangeFinder_MaxsonarXL(sonar_wall_analog_source,
            &front_sonar_mode_filter);
static LowPassFilterFloat front_sonar_lowpass_filter;           // Wall sonar lowpass filter      

static int16_t  front_sonar_control = 0;                        // Output to pitch control
static int16_t  front_sonar_raw = 0;                            // Raw sonar reading before filtration 
static float    front_sonar_filtered = 0;                       // Usable sonar reading after filtration
static float    front_sonar_distance_last = 0;                  // Distance we were from the wall last cycle, used to calculate speed.
static int16_t  front_sonar_distance_target = 75;               // Target distance to the wall, in cm.
static int16_t  front_sonar_distance_error = 0;                 // Difference between target distance and measured distance, in cm.
static int32_t  front_sonar_rate_target = 0;                    // The speed we want to approach the wall
static int32_t  front_sonar_rate_error = 0;                     // The speed error
static int32_t  front_sonar_rate_current = 0;                   // Current measured speed
static bool     front_sonar_control_saturated = false;          // for clamping the rate PID integrator when control is saturated
#define SONAR_POS_MAX_CONTROL 1000                               // Sets maximum output of the controller
#define FRONT_SONAR_MAX_RANGE 400                               // Sets maximum range for face-wall detection



#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


