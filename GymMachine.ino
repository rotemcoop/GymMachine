// ---------------------------------------------------------------------------------
// Wiring
// Teensy UART Tx to be connected to light blue next to black wire on Hoverboard
// ---------------------------------------------------------------------------------

typedef enum {
  MOTOR_RIGHT,
  MOTOR_LEFT
} motor_t;

typedef enum {
  HALL_RIGHT,
  HALL_LEFT
} hall_t;

typedef enum {
  CMD_PRF_CHANGE,
  CMD_STOP
} cmd_t;

// ---------------------------------------------------------------------------------
// --------------------------------- Golbal Data -----------------------------------
// ---------------------------------------------------------------------------------

// Pin 13 has an LED connected on most Arduino boards.
int led = 13;



// ---------------------------------------------------------------------------------
// ---------------------------- Serial Communication -------------------------------
// ---------------------------------------------------------------------------------

const uint8_t    GYRO_FRAME_LENGTH = 6;
const uint16_t   GYRO_FRAME_START = 256;
const uint8_t    GYRO_CONTACT_CLOSED_BYTE = 85;

void serial_write_frame(HardwareSerial* serial, int16_t value)
{
  // Set up a frame simulating gyroscope move on gyro daughterboard
  uint16_t uart_frame[GYRO_FRAME_LENGTH] = {
    GYRO_FRAME_START, 
    (uint8_t)(value & 0xff), 
    (uint8_t)(value >> 8 & 0xff), 
    (uint8_t)(value & 0xff), 
    (uint8_t)(value >> 8 & 0xff), 
    GYRO_CONTACT_CLOSED_BYTE };

  // Send a frame over the UART
  for(uint8_t i=0; i<6; i++) {
    serial->write9bit(uart_frame[i]); 
  } 
}  

//-------------------------------------------------------------------------

// Serial1 and Serial3 testing.
// Connect Serial1 Tx to Serial3 Rx
// Connect Serial1 Rx to Serial3 Tx

void serial1_to_serial3_test( uint16_t value )
{
  // Write a frame out of UART1
  serial_write_frame( &Serial1, value );

  delay (10);

  // Read fram from UART2 and print via USB
  int incomingByte;
  while (Serial3.available() > 0) {
    incomingByte = Serial3.read();
    Serial.print("UART received: ");
    Serial.println(incomingByte, DEC);
  }
}

// ---------------------------------------------------------------------------------
// ------------------------------------ Motor --------------------------------------
// ---------------------------------------------------------------------------------

inline void motor_right_torque( int16_t value ) {
  serial_write_frame( &Serial3, -value );
}

inline void motor_left_torque( int16_t value ) {
  serial_write_frame( &Serial1, -value );
}

inline void motor_both_torque( int16_t value ) {
  serial_write_frame( &Serial3, -value );
  serial_write_frame( &Serial1, -value );
}

// ---------------------------------------------------------------------------------

void motor_right_torque_smooth( int16_t value ) {
  static int16_t value_target = 0;
  static int16_t value_last = 0;
  value_target = value;
  if( value_last < value_target ) {
    value_last++;
  }
  else if( value_last > value_target ) {
    value_last--;    
  }
  motor_right_torque( value_last );
}

void motor_left_torque_smooth( int16_t value ) {
  static int16_t value_target = 0;
  static int16_t value_last = 0;
  value_target = value;
  if( value_last < value_target ) {
    value_last++;
  }
  else if( value_last > value_target ) {
    value_last--;    
  }
  motor_left_torque( value_last );
}

void motor_both_torque_smooth( int16_t value ) {
  motor_right_torque_smooth( value );
  motor_left_torque_smooth( value );
}

// ---------------------------------------------------------------------------------

void motor_wind_back( int torque )
{
  int right_ticks;
  int left_ticks;

  // Continue to apply torque as long as motors are turning.
  do {
    right_ticks = hall_right_ticks();
    left_ticks = hall_left_ticks();
    Serial.printf("Windback torque=%d\n", torque);
    for(int i=0; i<100; i++) {
      motor_both_torque( torque );
      hall_right_ticks();
      hall_left_ticks();
    }
  } while( right_ticks > hall_right_ticks() ||
           left_ticks > hall_left_ticks() );
  motor_both_torque( 0 );
}

//-------------------------------------------------------------------------

void motor_up_down_test( int max )
{
  // Loop n times and in between write same value to UART1 multiple times
  
  int i=0;
  const uint loop_cnt = 250;
  const uint loop_delay = 0;
  for(; i<max; i+=10)
  //for(; i<max; i+=1 )
  {
    Serial.printf("UART1 Sending: %d\n", i);
    Serial.println( "----------------------------" );
    for(int j=0; j<loop_cnt; j++) {
      delay(loop_delay);
      motor_both_torque( i );
      //hall_right_print();      
    }    
  }

  for(; i> -max; i-=10)
  {
    Serial.printf("UART1 Sending: %d\n", i);
    Serial.println( "----------------------------" );
    for(int j=0; j<loop_cnt; j++) {
      delay(loop_delay);
      motor_both_torque( i );
      //hall_right_print(); 
    }    
  }

  for(; i<0; i+=10)
  {
    Serial.printf("UART1 Sending: %d\n", i);
    Serial.println( "----------------------------" );
    for(int j=0; j<loop_cnt; j++) {
      delay(loop_delay);
      motor_both_torque( i );
      //hall_right_print();      
    }    
  }
}

// ---------------------------------------------------------------------------------
// -------------------------------- Hall Sensors -----------------------------------
// ---------------------------------------------------------------------------------

// LUT ( hall_tbl[prev_state][state] ) for mapping hall sensors transitions to ticks.
// +1 -> CW tick
// -1 -> CCW tick
// 00 -> no transition
// NA -> invalid transition
// There are 88-90 ticks per revolution
#define NA 99
const signed char hall_tbl[8][8] = { NA, NA, NA, NA, NA, NA, NA, NA,
                                     NA, 00, NA, -1, NA, +1, NA, NA,
                                     NA, NA, 00, +1, NA, NA, -1, NA,
                                     NA, +1, -1, 00, NA, NA, NA, NA,
                                     NA, NA, NA, NA, 00, -1, +1, NA,
                                     NA, -1, NA, NA, +1, 00, NA, NA,
                                     NA, NA, +1, NA, -1, NA, 00, NA,
                                     NA, NA, NA, NA, NA, NA, NA, NA };

// Enum representing hall sensors states
// MSB   = hall sensor 1
// Midel = hall sensor 2
// LSB   = hall sensor 3
// At any given time one or two (out of the three) sensors can have a value of 1.
// The three hall sensor wires should be connected such that the state tansirions
// match the enum order (from HALL_100 down to HALL101) when the wheel is turning CW.
enum {
  HALL_100 = 0b100,
  HALL_110 = 0b110,
  HALL_010 = 0b010,
  HALL_011 = 0b011,
  HALL_001 = 0b001,
  HALL_101 = 0b101
};                                    

// ---------------------------------------------------------------------------------

static uint hall_right_bad_state_cntr = 0;
static uint hall_left_bad_state_cntr = 0;
static int hall_right_ticks_cntr = 0;
static int hall_left_ticks_cntr = 0;
static int hall_right_ticks_per_sec_cntr = 0;
static int hall_left_ticks_per_sec_cntr = 0;

void hall_reset() {
  hall_right_bad_state_cntr = 0;
  hall_left_bad_state_cntr = 0;
  hall_right_ticks_cntr = 0;
  hall_left_ticks_cntr = 0;
  hall_right_ticks_per_sec_cntr = 0;
  hall_left_ticks_per_sec_cntr = 0;
}

/*
inline void hall_right_ticks_reset() {
  hall_right_ticks_cntr = 0;
}

inline void hall_left_ticks_reset() {
  hall_left_ticks_cntr = 0;
}
*/

inline unsigned long hall_right_ticks_per_sec() {
  return hall_right_ticks_per_sec_cntr;
}

inline unsigned long hall_left_ticks_per_sec() {
  return hall_left_ticks_per_sec_cntr;
}




// Return right motor ticks since start-up.
// This function has to be call frquent enugh to capture all hall sensor transitions.
// Blue wire -> pin14
// Green wire -> pin15
// Yellow wire -> pin16
//
int hall_right_ticks()
{
  static uint hall_state = 0;
  static uint hall_state_prev = 0;
  
  int h1 = digitalRead( 14 );
  int h2 = digitalRead( 15 );
  int h3 = digitalRead( 16 );
  hall_state_prev = hall_state;
  hall_state = h1 << 2 | h2 << 1 | h3;

  // Map hall sensors transition to rotation ticks (0, +1, -1)
  // Detect invalid transitions.
  int tick = hall_tbl[hall_state_prev][hall_state];
  if( tick == NA ) {
    hall_right_bad_state_cntr++;
    tick = 0;
  }
  hall_right_ticks_cntr += tick;

  //--------------------------------------------------------------------------------

  // Calculate ticks per second
  #define INTERVAL 100 // in millisec
  static int ticks_prev = hall_right_ticks_cntr;
  static unsigned long time_prev = millis();

  unsigned long time_curr = millis();
  if( time_curr >= time_prev + INTERVAL ) {
    hall_right_ticks_per_sec_cntr = (hall_right_ticks_cntr - ticks_prev) * (1000/INTERVAL);
    ticks_prev = hall_right_ticks_cntr;
    time_prev = time_curr;
  }

  //--------------------------------------------------------------------------------
  
  return hall_right_ticks_cntr;  
}

// ---------------------------------------------------------------------------------

// Return left motor ticks since start-up.
// This function has to be call frquent enugh to capture all hall sensor transitions.
// Blue wire -> pin18
// Green wire -> pin19
// Yellow wire -> pin20
//
int hall_left_ticks()
{
  static uint hall_state = 0;
  static uint hall_state_prev = 0;
  static uint bad_state_cntr = 0;

  int h1 = digitalRead( 18 );
  int h2 = digitalRead( 19 );
  int h3 = digitalRead( 20 );
  hall_state_prev = hall_state;
  hall_state = h1 << 2 | h2 << 1 | h3;

  // Map hall sensors transition to rotation ticks (0, +1, -1)
  // Detect invalid transitions.
  int tick = hall_tbl[hall_state_prev][hall_state];
  if( tick == NA ) {
    hall_left_bad_state_cntr++;
    tick = 0;
  }
  hall_left_ticks_cntr += tick;
  
  //--------------------------------------------------------------------------------

  // Calculate ticks per second
  #define INTERVAL 100 // in millisec
  static int ticks_prev = hall_left_ticks_cntr;
  static unsigned long time_prev = millis();

  unsigned long time_curr = millis();
  if( time_curr >= time_prev + INTERVAL ) {
    hall_left_ticks_per_sec_cntr = (hall_left_ticks_cntr - ticks_prev) * (1000/INTERVAL);
    ticks_prev = hall_left_ticks_cntr;
    time_prev = time_curr;
  }

  //--------------------------------------------------------------------------------
  
  return hall_left_ticks_cntr;  
}

// ---------------------------------------------------------------------------------

void hall_right_print()
{  
  int h1 = digitalRead(14);
  int h2 = digitalRead(15);
  int h3 = digitalRead(16);
  Serial.printf("Hall values: %d, %d, %d\n", h1, h2, h3);
}

// ---------------------------------------------------------------------------------

void hall_sensors_test()
{
  while( 1 ) {
    //motor_both_torque( 0 );
    Serial.printf( "right ticks/bad_ctr=%d/%d, left ticks/bad_ctr=%d/%d\n",\
                    hall_right_ticks(), hall_right_bad_state_cntr,\
                    hall_left_ticks(), hall_left_bad_state_cntr);
  }  
}

// ---------------------------------------------------------------------------------
// ---------------------------------- Workout --------------------------------------
// ---------------------------------------------------------------------------------

// Workout profile table.
// Specifies 8 bits resistance value for each cable pull distance in cm.

byte spring_prf[] = {   0,   1,   2,   3,   4,   5,   6,   7,   8,   9,
                       10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
                       20,  21,  22,  23,  24,  25,  26,  27,  28,  29,
                       30,  31,  32,  33,  34,  35,  36,  37,  38,  38,
                       40,  41,  42,  43,  44,  45,  46,  47,  48,  49,
                       50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
                       60,  61,  62,  63,  64,  65,  66,  67,  68,  69,
                       70,  71,  72,  73,  74,  75,  76,  77,  78,  79,
                       80,  81,  82,  83,  84,  85,  86,  87,  88,  89,
                       90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
                      100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
                      110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
                      120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
                      130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
                      140, 141, 142, 143, 144, 145, 146, 147, 148, 149 };

// ---------------------------------------------------------------------------------

#define W1 150
#define W2 150
#define W3 150
#define W4 150
byte weight_prf[] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,
                       W2,  W2,  W2,  W2,  W2,  W2,  W2,  W2,  W2,  W2,
                       W2,  W2,  W2,  W2,  W2,  W2,  W2,  W2,  W2,  W2,
                       W3,  W3,  W3,  W3,  W3,  W3,  W3,  W3,  W3,  W3,
                       W3,  W3,  W3,  W3,  W3,  W3,  W3,  W3,  W3,  W3,
                       W4,  W4,  W4,  W4,  W4,  W4,  W4,  W4,  W4,  W4,
                       W4,  W4,  W4,  W4,  W4,  W4,  W4,  W4,  W4,  W4 };

// ---------------------------------------------------------------------------------

// Geometry
#define TICKS_PER_ROTATION 89.0
#define WHEEL_DIAMETER 12.5
#define DIRECTION_COMP 100

byte* workout_prf = weight_prf;

void workout( byte prf_tbl[], int prf_tbl_len )
{
  // Aply torqueBased based on distance the cabled is pulled. 
  int right_distance_prev = 0;
  int left_distance_prev = 0;
  int right_speed = 0;
  int left_speed = 0;
  //int left_comp = 0;
  
  while( cmd_continue() )
  {
    int right_ticks = hall_right_ticks();
    int right_distance_raw = (int) ((PI * WHEEL_DIAMETER * right_ticks) / TICKS_PER_ROTATION);
    int right_distance = min( right_distance_raw, (prf_tbl_len - 1) );
    right_distance = max( right_distance, 0 );
    int right_torque = prf_tbl[ right_distance ] * 2;
    
    right_speed = hall_right_ticks_per_sec();
    if( right_speed > 0 ) {
      //right_torque *= 2;
      right_torque -= right_speed*2;
    }
    else if( right_speed < 0 ) {
      //right_torque -= right_speed;
      right_torque += DIRECTION_COMP;
    }

    if( right_distance <= 0 ) {
      right_torque = 0;
    }
    
    right_torque = max( right_torque, 0 );
        
    //--------------------------------------------------

    int left_ticks = hall_left_ticks();
    int left_distance_raw = (int) ((PI * WHEEL_DIAMETER * left_ticks) / TICKS_PER_ROTATION);
    int left_distance = min( left_distance_raw, (prf_tbl_len - 1) );
    left_distance = max( left_distance, 0 );
    int left_torque = prf_tbl[ left_distance ];
    
    left_speed = hall_left_ticks_per_sec();
    if( left_speed > 0 ) {
      //left_torque *= 2;
      left_torque -= left_speed*2;
    }
    else if( left_speed < 0 ) {
      left_torque -= left_speed;
      left_torque += DIRECTION_COMP;
    }

    if( left_distance <= 0 ) {
      left_torque = 0;
    }
    
    left_torque = max( left_torque, 0 );
    
    //--------------------------------------------------

    //right_torque *= 3;
    //left_torque *= 3;
    motor_right_torque_smooth( right_torque );
    motor_left_torque_smooth( left_torque );
        
    // Print out ticks, distance and torque.
    Serial.printf("R/L ticks=%d/%d, distance=%d/%d, torque=%d/%d, speed=%d/%d, ticks_per_sec=%d\n", \
                   right_ticks, left_ticks, right_distance, left_distance, right_torque, left_torque, \
                   right_speed, left_speed, hall_right_ticks_per_sec() );

    //Serial.printf("left_ticks=%d, left_distance=%u, left_torque=%d, lef_bad_state_cntr=%d \n", \
    //               left_ticks,    left_distance,    left_torque,    hall_left_bad_state_cntr );
  }
}

// ---------------------------------------------------------------------------------
// --------------------------------- Main Loop -------------------------------------
// ---------------------------------------------------------------------------------

inline void cmd_service_motor()
{
  hall_right_ticks();
  hall_left_ticks();
  motor_both_torque( 150 );  
}

// ---------------------------------------------------------------------------------

void cmd_wait_for_start()
{
  // Wait for cables pull.
  while( hall_right_ticks() == 0 && hall_left_ticks() == 0 ) {
    Serial.printf("Waiting for cables pull...\n");
    motor_both_torque( 0 );  
  }   
}

// ---------------------------------------------------------------------------------

//cmd_t cmd = CMD_PRF_CHANGE;

bool cmd_continue()
{
  if( Serial.available() ) {
    if( Serial.peek() == '\n' ) {
      Serial.read();
      return true;
    }
    return false;
  }
  return true;  
}

// ---------------------------------------------------------------------------------

void cmd_main()
{
  while(1)
  {
    motor_wind_back( 150 );
    hall_reset();
    int input = Serial.read();
    switch (input)
    {
      case '\n':
        break;
      
      case 'w':
        workout( weight_prf, sizeof(weight_prf) );
        break;

      case 's':
        workout( spring_prf, sizeof(spring_prf) );
        break;
        
      default:
        workout( weight_prf, sizeof(weight_prf) );
        break;
    }
  }
}


void setup() {
  // Initialize serial/USB communication at 9600 bits per second:
  Serial.begin(9600);
  
  // Initialize the UART1 and UART3 - 9 bits mode
  Serial1.begin (26300, SERIAL_9N1);
  Serial3.begin (26300, SERIAL_9N1);

  // Initialize hall sensors
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);

  // Hall sensor test pins
  //pinMode(21, INPUT_PULLUP);
  //pinMode(22, INPUT_PULLUP);
  //pinMode(23, INPUT_PULLUP);

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
}

//---------------------------------------------------------------------------

void loop()
{
  // Blink the LED
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  // motor_up_down_test( 200 );
  //hall_sensors_test();
  cmd_wait_for_start();
  motor_wind_back( 150 );
  hall_reset();
  //hall_right_ticks_reset();
  //hall_left_ticks_reset();
  workout( spring_prf, sizeof(spring_prf) );
  //workout( weight_prf, sizeof(weight_prf) );
  cmd_main();
  Serial.println( "----------------------------" );



  
  //int incomingByte;
    //while (Serial1.available() > 0) {
    //incomingByte = Serial1.read();
    //Serial.print("UART received: ");
    //Serial.println(incomingByte, DEC);
}
