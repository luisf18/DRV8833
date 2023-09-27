/* ====================================================
 *  Motor Library
 *    Author: luisf18 (github)
 *    Ver.: 1.0.0
 *    last_update: 12/03/2023 -> begin / Fix bip problem
 *    last_update: 15/02/2023
 *    created:     15/02/2023
 * 
 * from: "NEW_UNI9"
 * ====================================================
 */

#ifndef Motor_H
#define Motor_H
#include <Arduino.h>

typedef struct motor_move_t{
  int VL=0, VR=0, dt=0;
  void set(int vl, int vr, int duration){
    VL = vl;
    VR = vr;
    dt = duration;
  }
}motor_move_t;

class Motor{

  private:
    boolean OK = true;
    int p[4];
    uint32_t PWM_HZ  = 25000 ;
    uint8_t  PWM_RES = 10;
    uint16_t PWM_MAX = 1023;

    // Sound
    uint16_t SOUND_VOL = 20;
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

    Motor(uint8_t _in1, uint8_t _in2, uint8_t _in3, uint8_t _in4){
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
      
      Serial.println( ">> DRV8833 Begin" );
      Serial.printf(  "| - Pins: [ %d, %d, %d, %d ]\n",p[0],p[1],p[2],p[3] );
      Serial.print( "| - " );
      
      init_pwm( f_Hz , res );
      stop();
      return true;
    };

    boolean init_pwm( uint32_t HZ, uint8_t RES ){
      
      if( (RES == 0) || ( RES > 12 ) ) return false;
      PWM_RES = RES;
      PWM_MAX = ( 1 << RES ) - 1;
      
      PWM_HZ = ledcSetup(0, HZ, PWM_RES);
      ledcSetup(1, PWM_HZ, PWM_RES);
      ledcSetup(2, PWM_HZ, PWM_RES);
      ledcSetup(3, PWM_HZ, PWM_RES);
      ledcAttachPin(p[0], 0);
      ledcAttachPin(p[1], 1);
      ledcAttachPin(p[2], 2);
      ledcAttachPin(p[3], 3);
      Serial.printf( "PWM: [ Range: 0 to %d ] [ %d bits / %d Hz ]\n",PWM_MAX, PWM_RES, PWM_HZ );

      return true;
    };

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

    void off(){
      ledcWrite( 0, 0 );
      ledcWrite( 1, 0 );
      ledcWrite( 2, 0 );
      ledcWrite( 3, 0 );
      Speed[0] = 0;
      Speed[1] = 0;
    };

    void off( uint8_t motor ){
      if( motor > 1 ) return;
      ledcWrite( 0 + 2*motor, 0 );
      ledcWrite( 1 + 2*motor, 0 );
      Speed[motor] = 0;
    };
    
    void stop(){
      ledcWrite( 0, PWM_MAX );
      ledcWrite( 1, PWM_MAX );
      ledcWrite( 2, PWM_MAX );
      ledcWrite( 3, PWM_MAX );
      Speed[0] = 0;
      Speed[1] = 0;
    };
    
    void stop( uint8_t motor ){
      if( motor > 1 ) return;
      ledcWrite( 0 + 2*motor, PWM_MAX );
      ledcWrite( 1 + 2*motor, PWM_MAX );
      Speed[motor] = 0;
    };

    ////////////////////////////////////////////////////////////////////
    /////////////////////// LOWER LEVEL FUNCTIONS //////////////////////
    ////////////////////////////////////////////////////////////////////
    
    void write( int speed, int motor ){
      if( motor > 1  ) return;
      Speed[motor] = speed;
      if( speed >= 0 ){ ledcWrite( 0 + 2*motor,  speed ); ledcWrite( 1 + 2*motor, 0 ); }
      else            { ledcWrite( 1 + 2*motor, -speed ); ledcWrite( 0 + 2*motor, 0 ); }
    };

    void writeSD( int speed, int motor ){
      if( motor > 1  ) return;
      if( speed < 0 ){ ledcWrite( 0 + 2*motor, PWM_MAX+speed ); ledcWrite( 1 + 2*motor, PWM_MAX ); }
      else           { ledcWrite( 1 + 2*motor, PWM_MAX-speed ); ledcWrite( 0 + 2*motor, PWM_MAX ); }
    };

    
    /////////////////////////////////////////////////////////////////////
    ////////////////////////////  SOUND  ////////////////////////////////
    /////////////////////////////////////////////////////////////////////

    void bip( uint8_t n, uint16_t dt, uint32_t tone ){
      
      sound_tone( tone );
      uint16_t duty = sound_duty();
      //Serial.printf( "Sound >> Duty: %d\n", duty );

      for(int i=0;i<n;i++){
        move(duty,duty); delay( dt );
        off();           delay( dt );
      }

      sound_stop();

    };

    void bip( uint8_t n, uint16_t dt, uint32_t tone, uint8_t motor ){
      if( motor > 1 ) return;
      
      sound_tone( tone, motor );
      uint16_t duty = sound_duty();
      //Serial.printf( "Duty: %d\n", duty );

      for(int i=0;i<n;i++){
        write(duty,motor); delay( dt );
        off( motor );      delay( dt );
      }

      sound_stop( motor );
    };

    // Vol
    void     sound_vol( uint8_t vol ){ if( vol <= 25 ) SOUND_VOL = vol; };
    uint16_t sound_duty(){ return SOUND_VOL*(PWM_MAX/100); }

    // tone
    void sound_tone( uint32_t tone ){ sound_tone(tone,0); sound_tone(tone,1); };
    void sound_tone( uint32_t tone, uint8_t motor ){
      if( motor > 1 ) return;

      ledcSetup( 0 + 2*motor , tone, PWM_RES);
      ledcSetup( 1 + 2*motor , tone, PWM_RES);

      write( sound_duty(), motor );
    };

    // stop
    void sound_stop(){ sound_stop(0); sound_stop(1); };
    void sound_stop( uint8_t motor ){
      if( motor > 1 ) return;
      
      ledcSetup( 0 + 2*motor, PWM_HZ, PWM_RES);
      ledcSetup( 1 + 2*motor, PWM_HZ, PWM_RES);
      
      off(motor);
    };
    
};

#endif