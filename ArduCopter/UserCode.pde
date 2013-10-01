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
    
    
    /***** DEBUG VALUES 
    hal.console->println(s_min_far_away);
    hal.console->println(s_mid_far_away);
    hal.console->println(s_max_close);
    hal.console->println(s_mid_close);
    hal.console->println(s_distance_wall);
    hal.console->println(s_dz_wall);
    hal.console->println(s_sonar_reading);
    */
    //roll_pitch_mode == ROLL_PITCH_LOITER && 
    if (g.rc_6.control_in >= 700) {
        if (s_sonar_reading <= s_mid_close) { //first stage
              s_stage = 1;
              //The drone is a very dangerous zone, lets get it away as fast as possible
              // hal.console->println("first");
              pitch_control = map(s_sonar_reading,20,s_mid_close,300,300);
              //hal.console->println(s_mid_close); 
              // hal.console->println(pitch_control); 
           
        } else if (s_sonar_reading > s_mid_close && s_sonar_reading <= s_max_close) { //second stage 
             
              //The drone is going too close, lets push it back to the dz
              //  hal.console->println("second");
              if (s_stage == 0 || s_stage >= 3) { //kick back if is getting too close
                 pitch_control = 800;                   
              } else if (s_stage == 1) { //Kick forward to stop acceleration
                 pitch_control = -1400;
              } else {
                 //pitch_control = map(s_sonar_reading,s_mid_close,s_max_close,500,200);
                 pitch_control = 0;
              }
              s_stage = 2;
        } else if (s_sonar_reading  > s_max_close && s_sonar_reading <= (s_min_far_away)) { //fifth stage
              //Everything seems good, so dont touch pitch
              //hal.console->println("fifth");
              if (s_stage == 1 || s_stage == 2) {
                  pitch_control =  -1600;
                  s_stage = 0; 
              } else if (s_stage == 4 || s_stage == 5) { //coming from far away
                  pitch_control =  2600;
                  s_stage = 0; 
              } else {
                  s_stage = 0;
                  pitch_control = 0;
              }
        } else if (s_sonar_reading  > s_min_far_away && s_sonar_reading <= s_mid_far_away) { //thrid stage 
              //Drone a little bit far from the dz, trying to push back to get in the dz
              // hal.console->println("third");
              if (s_stage == 0) {
                 pitch_control = 1200;
              } else {
                 pitch_control = map(s_sonar_reading,s_min_far_away,s_mid_far_away,1000,-50);
              }
              s_stage = 3;
        } else if (s_sonar_reading >  s_mid_far_away) { //foufth stage
              //Drone too far from the dz pushing very hard to get back
              // hal.console->println("foufth");
              s_stage = 4;
              //pitch_control = map(s_sonar_reading,s_mid_far_away,(s_mid_far_away+s_distance_wall),-300,-400);
              pitch_control = -50;
        } else {
              //hal.console->println("nothing"); 
              s_stage = 0;
              pitch_control = 0;
        }
        //hal.console->println("MODE ON"); 
    } else {
        pitch_control=0;
        s_stage = 0;
    }
    Log_Write_Sonar(s_sonar_reading, pitch_control);
    
    //hal.console->println(s_sonar_reading);
    //hal.console->print(";");
    //hal.console->println(pitch_control);
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
