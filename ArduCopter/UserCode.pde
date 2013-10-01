/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    //AP_RANGEFINDER_MAXSONARXLL  AP_RANGEFINDER_MAXSONARXL
    sonarWall->calculate_scaler(AP_RANGEFINDER_MAXSONARXLL, 5);
}

int16_t map (int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
     return (x - in_min) * (out_max - out_min) / (in_max - in_min)  + out_min;
} 
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
   //Read Sonar on for the wall
   s_sonar_reading = sonarWall->read();
   //hal.console->print("SONAR READING");
   //hal.console->println(s_sonar_reading);
    
    if ( (roll_pitch_mode == ROLL_PITCH_STABLE) && (g.rc_6.control_in >= 700) && (s_sonar_reading < 200) ) {
       
       // Simple Distance ==> Pitch PID Controller        
       graffiti_distance_error = graffiti_distance_target - s_sonar_reading;                // Should return positive for too close, negative for too far away
       graffiti_control = g.pid_graffiti_distance.get_pid(graffiti_distance_error, G_Dt);   // Should return positive pitch (nose up) for too close, negative pitch (nose down) for too close
       graffiti_control = constrain_int16(graffiti_control, -4500, 4500);
       
    } else {
        
        g.pid_graffiti_distance.reset_I();       // Reset I-term
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
