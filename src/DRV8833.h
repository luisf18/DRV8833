/* ====================================================
 *  EasySensor Library
 *    Author: luisf18 (github)
 *    Ver.: 1.0.0
 *    last_update: 04/12/2022
 * ====================================================
 */

#ifndef DRV8833_H
#define DRV8833_H
#include <Arduino.h>

typedef struct motor_move_t{
  int VL=0, VR=0, dt=0;
  void set(int vl, int vr, int duration){
    VL = vl;
    VR = vr;
    dt = duration;
  }
}motor_move_t;

class DRV8833 {
  private:

    boolean OK = true;
    int p[4];
    uint32_t PWM_HZ  = 25000 ;
    uint8_t  PWM_RES = 10;
    uint16_t PWM_MAX = 1023;

    // Sound
    uint16_t SOUND_VOL = 15;
    //boolean  SOUND_ON[2]  = {false,false};

    // Moving
    boolean      Moving = false;
    uint32_t     Moving_timeout = 0;
    int          Moving_list = 0;
    motor_move_t Move_next;
    motor_move_t *Move_list;

    int   Speed[2] = {0,0};

  public:

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////// SETUP ///////////////////////////////
    ////////////////////////////////////////////////////////////////////
    
    DRV8833(uint8_t _in1, uint8_t _in2, uint8_t _in3, uint8_t _in4){
      p[0] = _in1;
      p[1] = _in2;
      p[2] = _in3;
      p[3] = _in4;
    };

    boolean begin(){ return begin( PWM_HZ, PWM_RES ); };
    
    boolean begin(uint32_t f_Hz, uint8_t res){
      
      pinMode(p[0],OUTPUT);
      pinMode(p[1],OUTPUT);
      pinMode(p[2],OUTPUT);
      pinMode(p[3],OUTPUT);
      
      Serial.println( "[x] DRV8833 Begin" );
      Serial.printf(  " | - Pins: [ %d, %d, %d, %d ]\n",p[0],p[1],p[2],p[3] );
      Serial.printf(  " | - PWM: [ Range: 0 to %d ] [ %d bits / %d Hz ]\n",PWM_MAX, PWM_RES, PWM_HZ );
      
      init_pwm( f_Hz , res );
      stop();

      return true;

    };

    uint8_t init_pwm( uint32_t HZ, uint8_t RES ){

      #ifdef ESP32
      if( (RES > 0) && ( RES <= 12 ) ) PWM_RES = RES;
      PWM_MAX = ( 1 << PWM_RES ) - 1;
      PWM_HZ  = ledcSetup(0, HZ, PWM_RES);
      ledcSetup(1, PWM_HZ, PWM_RES);
      ledcSetup(2, PWM_HZ, PWM_RES);
      ledcSetup(3, PWM_HZ, PWM_RES);
      ledcAttachPin(p[0], 0);
      ledcAttachPin(p[1], 1);
      ledcAttachPin(p[2], 2);
      ledcAttachPin(p[3], 3);

      #else

      // update resolution
        if( RES >= 4 && RES <= 16 )
          PWM_RES = RES;
        analogWriteResolution(PWM_RES);
      
      // update range
        PWM_MAX = (1 << PWM_RES) - 1;

      // update freq
        if( HZ >= 100 && HZ <= 60000)
          PWM_HZ = HZ;
        analogWriteFreq(PWM_HZ);
      
      #endif
      
      Serial.printf( "PWM: [ Range: 0 to %d ] [ %d bits / %d Hz ]\n",PWM_MAX, PWM_RES, PWM_HZ );
      
      // Retorna uma "nota" do que foi feito
      return ( true | (PWM_HZ == HZ) << 1 | (PWM_RES == RES) << 2 );

    };

    int pin(uint8_t n){
      return ( n > 3 ? 0 : p[n] );
    };

    int range(){
      return PWM_MAX;
    };

    ////////////////////////////////////////////////////////////////////
    ////////// HIGH LEVEL MOVE - DIFFERENTIAL DRIVE ROBOT //////////////
    ////////////////////////////////////////////////////////////////////

    void diff_drive(int linear, int angular, boolean invert, boolean radio_range ){
      
      if( radio_range ){
        linear  = map( linear,  1000, 2000, -PWM_MAX, PWM_MAX );
        angular = map( angular, 1000, 2000, -PWM_MAX, PWM_MAX );
      }

      int vel_l  = constrain( linear + angular, -PWM_MAX, PWM_MAX );
      int vel_r  = constrain( linear - angular, -PWM_MAX, PWM_MAX );
      
      if(invert) move( -vel_r, -vel_l ); 
      else       move(  vel_l,  vel_r );

    }

    ////////////////////////////////////////////////////////////////////
    //////////////////// HIGH LEVEL MOVE FUNCTIONS /////////////////////
    ////////////////////////////////////////////////////////////////////
    
    int read(uint8_t n){
      if(n > 1) return 0;
      return Speed[n];
    };

    void clear_moving(){ Moving = false; Moving_list = 0; };

    // motor_move_t
    void move( motor_move_t m ){ move(m.VL,m.VR); Moving_timeout = millis() + m.dt; Moving = true; }
    void move( motor_move_t *m, int len ){
      move(*m);
      Move_list = m+1;
      Moving_list = len-1; // o primeiro ja esta em execução
    }

    void move_for( int speed_0, int speed_1, uint32_t duration ){
      Moving = true;
      Moving_timeout = millis() + duration;
      move(speed_0, speed_1);
      Moving_list = 0;
    };

    void move_for_then( int speed_0, int speed_1, uint32_t duration, int speed_next_0, int speed_next_1, uint32_t duration_next ){
      Moving = true;
      Moving_timeout = millis() + duration;
      move(speed_0, speed_1);
      // Next
      Moving_list = 1;
      Move_next.VL = speed_next_0;
      Move_next.VR = speed_next_1;
      Move_next.dt = duration_next;
      Move_list = &Move_next;
    };

    int update(){
      if( !Moving ) return -1;

      if(millis() >= Moving_timeout ){
        Moving = false;
        if( Moving_list ){
          Moving_list--;
          Moving  = true;
          Moving_timeout += Move_list->dt;
          if( abs(Move_list->VL) <= PWM_MAX ) write(Move_list->VL,0); else stop(0);
          if( abs(Move_list->VR) <= PWM_MAX ) write(Move_list->VR,1); else stop(1);
          Move_list++;
        }
      }

      return ( Moving + Moving_list );

    };

    void move(   int speed_0, int speed_1 ){ write(   speed_0, 0 ); write(   speed_1, 1 ); };
    void moveSD( int speed_0, int speed_1 ){ writeSD( speed_0, 0 ); writeSD( speed_1, 1 ); };
    void moveRaw(   int speed_0, int speed_1 ){ writeRaw(   speed_0, 0 ); writeRaw(   speed_1, 1 ); };

    void off(){
      #ifdef ESP32
      ledcWrite( 0, 0 );
      ledcWrite( 1, 0 );
      ledcWrite( 2, 0 );
      ledcWrite( 3, 0 );
      #else
      analogWrite( p[0], 0 );
      analogWrite( p[1], 0 );
      analogWrite( p[2], 0 );
      analogWrite( p[3], 0 );
      #endif
      Speed[0] = 0;
      Speed[1] = 0;
    };

    void off( uint8_t motor ){
      if( motor > 1 ) return;
      #ifdef ESP32
      ledcWrite( 0 + 2*motor, 0 );
      ledcWrite( 1 + 2*motor, 0 );
      #else
      analogWrite( p[0+2*motor], 0 );
      analogWrite( p[1+2*motor], 0 );
      #endif
      Speed[motor] = 0;
    };
    
    void stop(){
      #ifdef ESP32
      ledcWrite( 0, PWM_MAX );
      ledcWrite( 1, PWM_MAX );
      ledcWrite( 2, PWM_MAX );
      ledcWrite( 3, PWM_MAX );
      #else
      analogWrite( p[0], PWM_MAX );
      analogWrite( p[1], PWM_MAX );
      analogWrite( p[2], PWM_MAX );
      analogWrite( p[3], PWM_MAX );
      #endif
      Speed[0] = 0;
      Speed[1] = 0;
    };
    
    void stop( uint8_t motor ){
      if( motor > 1 ) return;
      #ifdef ESP32
      ledcWrite( 0 + 2*motor, PWM_MAX );
      ledcWrite( 1 + 2*motor, PWM_MAX );
      #else
      analogWrite( p[0+2*motor], PWM_MAX );
      analogWrite( p[1+2*motor], PWM_MAX );
      #endif
      Speed[motor] = 0;
    };

    ////////////////////////////////////////////////////////////////////
    /////////////////////// LOWER LEVEL FUNCTIONS //////////////////////
    ////////////////////////////////////////////////////////////////////

    void writeRaw( int speed, int motor ){
      if( motor > 1  ) return;
      #ifdef ESP32
      if( speed >= 0 ){ ledcWrite( 0 + 2*motor,  speed ); ledcWrite( 1 + 2*motor, 0 ); }
      else            { ledcWrite( 1 + 2*motor, -speed ); ledcWrite( 0 + 2*motor, 0 ); }
      #else
      motor = 2*motor;
      if( speed >= 0 ){ analogWrite( p[0+motor],  speed ); analogWrite( p[1+motor], 0 ); }
      else            { analogWrite( p[1+motor], -speed ); analogWrite( p[0+motor], 0 ); }
      #endif
    };

    void write( int speed, int motor ){
      if( motor > 1  ) return;
      Speed[motor] = speed;
      #ifdef ESP32
      if( speed >= 0 ){ ledcWrite( 0 + 2*motor,  speed ); ledcWrite( 1 + 2*motor, 0 ); }
      else            { ledcWrite( 1 + 2*motor, -speed ); ledcWrite( 0 + 2*motor, 0 ); }
      #else
      motor = 2*motor;
      if( speed >= 0 ){ analogWrite( p[0+motor],  speed ); analogWrite( p[1+motor], 0 ); }
      else            { analogWrite( p[1+motor], -speed ); analogWrite( p[0+motor], 0 ); }
      #endif
    };

    void writeSD( int speed, int motor ){
      Speed[motor] = speed;
      if( motor > 1  ) return;
      #ifdef ESP32
      if( speed < 0 ){ ledcWrite( 0 + 2*motor, PWM_MAX+speed ); ledcWrite( 1 + 2*motor, PWM_MAX ); }
      else           { ledcWrite( 1 + 2*motor, PWM_MAX-speed ); ledcWrite( 0 + 2*motor, PWM_MAX ); }
      #else
      motor = 2*motor;
      if( speed < 0 ){ analogWrite( p[0+motor], PWM_MAX+speed ); analogWrite( p[1+motor], PWM_MAX ); }
      else           { analogWrite( p[1+motor], PWM_MAX-speed ); analogWrite( p[0+motor], PWM_MAX ); }
      #endif
    };

    /////////////////////////////////////////////////////////////////////
    ////////////////////////////  SOUND  ////////////////////////////////
    /////////////////////////////////////////////////////////////////////

    // Vol

    void     sound_vol( uint8_t vol ){ if( vol <= 25 ) SOUND_VOL = vol; };
    uint16_t sound_duty(){ return SOUND_VOL*(PWM_MAX/100); }

    // Bip

    void bip( uint8_t n, uint16_t dt, uint32_t tone ){

      sound_tone( tone );
      uint16_t duty = sound_duty();

      for(int i=0;i<n;i++){
        moveRaw(duty,duty); delay( dt );
        moveRaw(   0,   0); delay( dt );
      }

      sound_stop();

    };

    void bip( uint8_t n, uint16_t dt, uint32_t tone, uint8_t motor ){
      if( motor > 1 ) return;
      
      sound_tone( tone, motor );
      uint16_t duty = sound_duty();

      for(int i=0;i<n;i++){
        writeRaw(duty,motor); delay( dt );
        writeRaw(   0,motor ); delay( dt );
      }

      sound_stop( motor );
    };

    // tone
    void sound_tone( uint32_t tone ){
      sound_tone(tone,0);
      sound_tone(tone,1);
    };

    void sound_tone( uint32_t tone, uint8_t motor ){
      if( motor > 1 ) return;
      #ifdef ESP32
      ledcSetup( 0 + 2*motor , tone, PWM_RES);
      ledcSetup( 1 + 2*motor , tone, PWM_RES);
      #else
      analogWriteFreq(tone);
      #endif
      writeRaw( sound_duty(), motor );
    };

    // stop
    void sound_stop(){
      sound_stop(0);
      sound_stop(1);
    };

    void sound_stop( uint8_t motor ){
      if( motor > 1 ) return;
      #ifdef ESP32
      ledcSetup( 0 + 2*motor, PWM_HZ, PWM_RES);
      ledcSetup( 1 + 2*motor, PWM_HZ, PWM_RES);
      #else
      analogWriteFreq(PWM_HZ);
      #endif
      write( Speed[motor], motor );
    };
    

};



#endif