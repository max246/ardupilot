/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    //AP_RANGEFINDER_MAXSONARXLL  AP_RANGEFINDER_MAXSONARXL
    sonarWall->calculate_scaler(AP_RANGEFINDER_MAXSONARXLL, 5);
    sonar_wall_lowpass_filter.set_cutoff_frequency(0.01f, 5.0f);       // Initialize filter with 0.01 second time step (100Hz) and cutoff frequency of 5Hz
}

int16_t map (int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
     return (x - in_min) * (out_max - out_min) / (in_max - in_min)  + out_min;
} 
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
   //Read Sonar on for the wall
   s_sonar_raw = sonarWall->read();
   s_sonar_reading = sonar_wall_lowpass_filter.apply(s_sonar_raw);
   //hal.console->print("SONAR READING");
   //hal.console->println(s_sonar_reading);
    
    if ( (roll_pitch_mode == ROLL_PITCH_STABLE) && (g.rc_6.control_in >= 700) && (s_sonar_reading < 200) ) {
       
        // Distance ==> Rate PID Controller
        graffiti_distance_error = graffiti_distance_target - s_sonar_reading;               // Should return positive for too close, negative for too far away
        graffiti_rate_target = g.pi_graffiti_distance.get_p(graffiti_distance_error);       // Should return a target speed, positive away from wall, negative towards wall.
        graffiti_rate_target = constrain_int32(graffiti_rate_target, -100, 100);            // Constrain target to 100 cm/s  for sanity.
        
        // Rate ==> Pitch PID Controller  
        graffiti_rate_current = (s_sonar_reading - graffiti_distance_last)*100;             // Current speed, in cm/second. Positive away from wall. Negative towards wall.
        graffiti_rate_error = graffiti_rate_target - graffiti_rate_current;                 // Speed Error, in cm/seconds. Positive away from wall.
        graffiti_control = g.pid_graffiti_rate.get_pid(graffiti_rate_error, G_Dt);          // Should return positive pitch (nose up) to accelerate away from wall.
        graffiti_control = constrain_int16(graffiti_control, -500, 500);                    // Constrain to reasonable numbers.
        graffiti_distance_last = s_sonar_reading;                                           // Save current distance for next iteration.
       
    } else {
        
        g.pid_graffiti_rate.reset_I();                      // Reset I-term
        graffiti_distance_last = s_sonar_reading;           // Reset this to something reasonable. To-Do: we should handle on/off switching better.
        graffiti_control=0;

    }
    
    Log_Write_Sonar(s_sonar_reading, graffiti_control);
    
    //hal.console->println(s_sonar_reading);
    //hal.console->print(";");
    //hal.console->println(graffiti_control);
    //hal.console->println(g.rc_2.control_in);
    
    //current_pitch = g.rc_2.control_in;
    
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
