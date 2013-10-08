/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    FrontSonar->calculate_scaler(AP_RANGEFINDER_MAXSONARXLL, 5);
    SideSonar->calculate_scaler(AP_RANGEFINDER_MAXSONARXLL, 5);
    front_sonar_lowpass_filter.set_cutoff_frequency(0.01f, 2.0f);                                               // Initialize filter with 0.01 second time step (100Hz) and cutoff frequency of 5Hz
    side_sonar_lowpass_filter.set_cutoff_frequency(0.01f, 2.0f);
}

int16_t map (int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
     return (x - in_min) * (out_max - out_min) / (in_max - in_min)  + out_min;
} 
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
   //Read Sonar on for the wall
   front_sonar_raw = FrontSonar->read();
   side_sonar_raw = FrontSonar->read();
   front_sonar_filtered = front_sonar_lowpass_filter.apply( (float) front_sonar_raw );
   side_sonar_filtered = side_sonar_lowpass_filter.apply( (float) side_sonar_raw );
    
    if ( (roll_pitch_mode == ROLL_PITCH_STABLE) && (g.rc_6.control_in >= 700) && (front_sonar_filtered < FRONT_SONAR_MAX_RANGE) ) {
       
        // Distance ==> Rate PID Controller
        front_sonar_distance_error = front_sonar_distance_target - (int16_t)front_sonar_filtered;               // Return positive for too close, negative for too far away
        front_sonar_rate_target = g.pi_sonar_distance.get_p(front_sonar_distance_error);                        // Return a target speed, positive away from wall, negative towards wall.
        front_sonar_rate_target = constrain_int32(front_sonar_rate_target, -100, 100);                          // Constrain target to 100 cm/s  for sanity.
        
        // Rate ==> Pitch PID Controller  
        front_sonar_rate_current = (float)((front_sonar_filtered - front_sonar_distance_last)*100.0f);          // Current speed, in cm/second. Positive away from wall. Negative towards wall.
        front_sonar_rate_error = front_sonar_rate_target - front_sonar_rate_current;                            // Speed Error, in cm/seconds. Positive away from wall.
        front_sonar_control = g.pid_front_sonar_rate.get_p(front_sonar_rate_error);                             // Return positive pitch (nose up) to accelerate away from wall.
        front_sonar_control += g.pid_front_sonar_rate.get_d(front_sonar_rate_error, G_Dt);
        
        if (front_sonar_control_saturated){                                                                     // If control is saturated
            front_sonar_control += g.pid_front_sonar_rate.get_integrator();                                     // Use clamped integrator
        } else {                                                                                                // Control is not saturated
            front_sonar_control += g.pid_front_sonar_rate.get_i(front_sonar_rate_error, G_Dt);                  // Use live integrator
        }
        
        if (labs(front_sonar_control) > SONAR_POS_MAX_CONTROL){                                                 // Constrain to reasonable numbers.
            front_sonar_control_saturated = true;                                                               // Clamp integrator
            front_sonar_control = constrain_int16(front_sonar_control, -SONAR_POS_MAX_CONTROL, SONAR_POS_MAX_CONTROL);  
        } else {
            front_sonar_control_saturated = false;                                                              // Unclamp integrator
        }
        
    } else if ( (roll_pitch_mode == ROLL_PITCH_LOITER) && (g.rc_6.control_in >= 700) && (front_sonar_filtered < FRONT_SONAR_MAX_RANGE) ) {
    
        // Distance ==> Rate PID Controller
        front_sonar_distance_error = front_sonar_distance_target - (int16_t)front_sonar_filtered;               // Return positive for too close, negative for too far away
        front_sonar_rate_target = g.pi_sonar_l_dist.get_p(front_sonar_distance_error);                          // Return a target speed, positive away from wall, negative towards wall.
        front_sonar_control = constrain_int32(front_sonar_rate_target, -2000, 2000);                            // Constrain target to 200 cm/s  for sanity.
        
        g.pid_front_sonar_rate.reset_I();                                                                       // Reset I-term
        front_sonar_control_saturated = false;                                                                  // Unclamp integrator
       
    } else {
        
        g.pid_front_sonar_rate.reset_I();                                                                       // Reset I-term
        front_sonar_control_saturated = false;                                                                  // Unclamp integrator
        front_sonar_control=0;        
    }
    
    front_sonar_distance_last = front_sonar_filtered;
    
    if ( (roll_pitch_mode == ROLL_PITCH_STABLE) && (g.rc_6.control_in >= 700) && (side_sonar_filtered < SIDE_SONAR_MAX_RANGE) ) {
       
        // Distance ==> Rate PID Controller
        side_sonar_distance_error = side_sonar_distance_target - (int16_t)side_sonar_filtered;                  // Return positive for too close, negative for too far away
        side_sonar_rate_target = g.pi_sonar_distance.get_p(side_sonar_distance_error);                          // Return a target speed, positive away from wall, negative towards wall.
        side_sonar_rate_target = constrain_int32(side_sonar_rate_target, -100, 100);                            // Constrain target to 100 cm/s  for sanity.
        
        // Rate ==> Pitch PID Controller  
        side_sonar_rate_current = (float)((side_sonar_filtered - side_sonar_distance_last)*100.0f);             // Current speed, in cm/second. Positive away from wall. Negative towards wall.
        side_sonar_rate_error = side_sonar_rate_target - side_sonar_rate_current;                               // Speed Error, in cm/seconds. Positive away from wall.
        side_sonar_control = g.pid_side_sonar_rate.get_p(side_sonar_rate_error);                                // Return positive pitch (nose up) to accelerate away from wall.
        side_sonar_control += g.pid_side_sonar_rate.get_d(side_sonar_rate_error, G_Dt);
        
        if (side_sonar_control_saturated){                                                                      // If control is saturated
            side_sonar_control += g.pid_side_sonar_rate.get_integrator();                                       // Use clamped integrator
        } else {                                                                                                // Control is not saturated
            side_sonar_control += g.pid_side_sonar_rate.get_i(side_sonar_rate_error, G_Dt);                     // Use live integrator
        }
        
        if (labs(side_sonar_control) > SONAR_POS_MAX_CONTROL){                                                  // Constrain to reasonable numbers.
            side_sonar_control_saturated = true;                                                                // Clamp integrator
            side_sonar_control = constrain_int16(side_sonar_control, -SONAR_POS_MAX_CONTROL, SONAR_POS_MAX_CONTROL);  
        } else {
            side_sonar_control_saturated = false;                                                               // Unclamp integrator
        }
        
        side_sonar_control *= SONAR_SIDE;                                                                       // Flip control direction for right side sonars
                
    } else if ( (roll_pitch_mode == ROLL_PITCH_LOITER) && (g.rc_6.control_in >= 700) && (side_sonar_filtered < SIDE_SONAR_MAX_RANGE) ) {
    
        // Distance ==> Rate PID Controller
        side_sonar_distance_error = side_sonar_distance_target - (int16_t)side_sonar_filtered;                  // Return positive for too close, negative for too far away
        side_sonar_rate_target = g.pi_sonar_l_dist.get_p(side_sonar_distance_error);                            // Return a target speed, positive away from wall, negative towards wall.
        side_sonar_control = constrain_int32(side_sonar_rate_target, -2000, 2000);                              // Constrain target to 200 cm/s  for sanity.
        side_sonar_control *= SONAR_SIDE;                                                                       // Flip control direction for right side sonars
        
        g.pid_side_sonar_rate.reset_I();                                                                        // Reset I-term
        side_sonar_control_saturated = false;                                                                   // Unclamp integrator
       
    } else {
        
        g.pid_side_sonar_rate.reset_I();                                                                        // Reset I-term
        side_sonar_control_saturated = false;                                                                   // Unclamp integrator
        side_sonar_control=0;        
    }
    
    side_sonar_distance_last = side_sonar_filtered;
    
    Log_Write_Sonar(front_sonar_filtered, front_sonar_control);
    
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
