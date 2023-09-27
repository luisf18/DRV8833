/* ====================================================
 *  Motor Library
 *    Author: luisf18 (github)
 *    Ver.: 1.0.0
 *    last_update: 12/03/2023 -> begin / Fix bip problem
 *    last_update: 15/02/2023
 *    created:     15/02/2023
 * ====================================================
 */

#ifndef Motor_H
#define Motor_H
#include <Arduino.h>

class Motor{

  private:
    boolean OK = true;
    int p[4];
    uint32_t PWM_HZ  = 25000;
    uint8_t  PWM_RES = 8;
    uint16_t PWM_MAX = 255;

    // Sound
    uint16_t SOUND_VOL = 15;

    // Path
    int      *path_speed_0;
    int      *path_speed_1;
    uint16_t *path_time;
    uint32_t  path_timeout;
    int       path_len;
    int       path_index;
    boolean   path_on = false;

    int   Speed[2] = {0,0};

  public:

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////// SETUP ///////////////////////////////
    ////////////////////////////////////////////////////////////////////

    Motor(uint8_t _in1, uint8_t _in2, uint8_t _in3, uint8_t _in4){
      p[0] = _in1;
      p[1] = _in2;
      p[2] = _in3;
      p[3] = _in4;
    };
    
    boolean begin(){ return begin( PWM_HZ ); };
    boolean begin(uint32_t f_Hz){
      pinMode(p[0],OUTPUT);
      pinMode(p[1],OUTPUT);
      pinMode(p[2],OUTPUT);
      pinMode(p[3],OUTPUT);
      analogWriteFreq(f_Hz);
      PWM_HZ = f_Hz;
      
      Serial.println( "[x] DRV8833 Begin" );
      Serial.printf(  " | - Pins: [ %d, %d, %d, %d ]\n",p[0],p[1],p[2],p[3] );
      Serial.printf(  " | - PWM: [ Range: 0 to %d ] [ %d bits / %d Hz ]\n",PWM_MAX, PWM_RES, PWM_HZ );

      stop();
      return true;
    };

    ////////////////////////////////////////////////////////////////////
    //////////////////// HIGH LEVEL MOVE FUNCTIONS /////////////////////
    ////////////////////////////////////////////////////////////////////
    
    int read(uint8_t n){
      if(n > 1) return 0;
      return Speed[n];
    };

    void move(   int speed_0, int speed_1 ){ write(   speed_0, 0 ); write(   speed_1, 1 ); };
    void moveSD( int speed_0, int speed_1 ){ writeSD( speed_0, 0 ); writeSD( speed_1, 1 ); };

    void off(){
      analogWrite( p[0], 0 );
      analogWrite( p[1], 0 );
      analogWrite( p[2], 0 );
      analogWrite( p[3], 0 );
      Speed[0] = 0;
      Speed[1] = 0;
    };

    void off( uint8_t motor ){
      if( motor > 1 ) return;
      Speed[motor] = 0;
      motor = 2*motor;
      analogWrite( p[0+motor], 0 );
      analogWrite( p[1+motor], 0 );
    };
    
    void stop(){
      analogWrite( p[0], PWM_MAX );
      analogWrite( p[1], PWM_MAX );
      analogWrite( p[2], PWM_MAX );
      analogWrite( p[3], PWM_MAX );
      Speed[0] = 0;
      Speed[1] = 0;
    };
    
    void stop( uint8_t motor ){
      if( motor > 1 ) return;
      Speed[motor] = 0;
      motor = 2*motor;
      analogWrite( p[0+motor], PWM_MAX );
      analogWrite( p[1+motor], PWM_MAX );
    };

    ////////////////////////////////////////////////////////////////////
    /////////////////////// LOWER LEVEL FUNCTIONS //////////////////////
    ////////////////////////////////////////////////////////////////////
    
    void write( int speed, int motor ){
      if( motor > 1  ) return;
      Speed[motor] = speed;
      motor = 2*motor;
      if( speed >= 0 ){ analogWrite( p[0+motor],  speed ); analogWrite( p[1+motor], 0 ); }
      else            { analogWrite( p[1+motor], -speed ); analogWrite( p[0+motor], 0 ); }
    };

    void writeSD( int speed, int motor ){
      if( motor > 1  ) return;
      motor = 2*motor;
      if( speed < 0 ){ analogWrite( p[0+motor], PWM_MAX+speed ); analogWrite( p[1+motor], PWM_MAX ); }
      else           { analogWrite( p[1+motor], PWM_MAX-speed ); analogWrite( p[0+motor], PWM_MAX ); }
    };

    
    /////////////////////////////////////////////////////////////////////
    ////////////////////////////  SOUND  ////////////////////////////////
    /////////////////////////////////////////////////////////////////////

    void bip( uint8_t n, uint16_t dt, uint32_t tone ){
      
      analogWriteFreq(tone);
      uint16_t duty = sound_duty();

      for(int i=0;i<n;i++){
        move(duty,duty); delay( dt );
        off();           delay( dt );
      }

      sound_stop();

    };

    void bip( uint8_t n, uint16_t dt, uint32_t tone, uint8_t motor ){
      if( motor > 1 ) return;
      
      analogWriteFreq(tone);
      uint16_t duty = sound_duty();

      for(int i=0;i<n;i++){
        write(duty,motor); delay( dt );
        off( motor );      delay( dt );
      }

      sound_stop();
    };

    // Vol
    void     sound_vol( uint8_t vol ){ if( vol <= 25 ) SOUND_VOL = vol; };
    uint16_t sound_duty(){ return SOUND_VOL*(PWM_MAX/100); }

    // stop
    void sound_stop(){ analogWriteFreq(PWM_HZ); };
    
};

#endif