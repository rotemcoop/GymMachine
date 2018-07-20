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
  DIRECTION_PULL,
  DIRECTION_REL
} direction_t;

typedef enum {
  CMD_PRF_CHANGE,
  CMD_STOP
} cmd_t;

typedef struct {
  char*   name;
  int     add_pull;
  int     add_rel;
  int     mult_pull;
  int     mult_rel;
  uint    len;
  byte*   tbl;
} workout_prf_t;

// ---------------------------------------------------------------------------------
// ---------------------------- Serial Communication -------------------------------
// ---------------------------------------------------------------------------------

const uint8_t    SERIAL_FRAME_LENGTH = 6;
const uint16_t   SERIAL_FRAME_START = 256;
const uint8_t    SERIAL_CONTACT_CLOSED_BYTE = 85;

void serialWriteFrame( HardwareSerial* serial, int16_t value )
{
  // Set up a frame
  uint16_t uartFrame[ SERIAL_FRAME_LENGTH ] = {
    SERIAL_FRAME_START, 
    (uint8_t)(value & 0xff), 
    (uint8_t)(value >> 8 & 0xff), 
    (uint8_t)(value & 0xff), 
    (uint8_t)(value >> 8 & 0xff), 
    SERIAL_CONTACT_CLOSED_BYTE };

  // Send the frame over the UART
  for( uint8_t i=0; i<6; i++ ) {
    serial->write9bit( uartFrame[i] ); 
  } 
}  

//-------------------------------------------------------------------------

// Serial1 and Serial3 testing.
// Connect Serial1 Tx to Serial3 Rx
// Connect Serial1 Rx to Serial3 Tx

void serial1ToSerial3Test( uint16_t value )
{
  // Write a frame out of UART1
  serialWriteFrame( &Serial1, value );

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
// -------------------------------- Hall Sensors -----------------------------------
// ---------------------------------------------------------------------------------

// Right motor hall wires to Teensy 3.2 pins
// Blue wire -> pin14
// Green wire -> pin15
// Yellow wire -> pin16
//
// Left motor hall wires to Teensy 3.2 pins
// Blue wire -> pin18
// Green wire -> pin19
// Yellow wire -> pin20//

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

// Interval (in millisec) to compute speed and acceleration
#define HALL_INTERVAL 50

class Hall {
  private:
  int h1Pin;
  int h2Pin;
  int h3Pin;
  uint state;
  uint statePrev;
  uint badStateCntr;
  int ticksCntr;
  int speedCntr;
  int accelCntr;
  struct {
    unsigned long time;
    int ticks;
    int speed;
  } prev;  
      
  public:
  Hall( int h1PinPrm, int h2PinPrm, int h3PinPrm ) :
    h1Pin( h1PinPrm ),
    h2Pin( h2PinPrm ),
    h3Pin( h3PinPrm ) {
      reset();
      state = 0;
      statePrev = 0;
      prev.time = millis();
      prev.ticks = 0;
      prev.speed = 0;
  }

  //--------------------------------------------------------------------------------
  
  void reset() {
    badStateCntr = 0;
    ticksCntr = 0;
    speedCntr = 0;
    accelCntr = 0;  
  }

 //--------------------------------------------------------------------------------
 
  // Return hall sensor ticks since reset.
  // This function has to be call frquent enugh to capture all hall sensor transitions.
  // Blue wire -> h1Pin
  // Green wire -> h2Pin
  // Yellow wire -> h3Pin
  //
  int ticks() {
    int h1 = digitalRead( h1Pin );
    int h2 = digitalRead( h2Pin );
    int h3 = digitalRead( h3Pin );
    statePrev = state;
    state = h1 << 2 | h2 << 1 | h3;

    // Map hall sensors transition to rotation ticks (0, +1, -1)
    // Detect invalid transitions.
    int tick = hall_tbl[statePrev][state];
    if( tick == NA ) {
      badStateCntr++;
      tick = 0;
    }
    ticksCntr += tick;

    //--------------------------------------------------------------------------------

    // Calculate ticks spped and acceleration
    unsigned long timeCurr = millis();
    if( timeCurr >= prev.time + HALL_INTERVAL ) {
      speedCntr = (speedCntr + (int)((ticksCntr - prev.ticks) * (1000/HALL_INTERVAL)))/2;
      accelCntr = (3*accelCntr + (int)((speedCntr - prev.speed) * (1000/HALL_INTERVAL)))/4;
      prev.ticks = ticksCntr;
      prev.speed = speedCntr;
      prev.time = timeCurr;
    }

    return ticksCntr; 
  }
  
  inline uint badState() {
    return badStateCntr;
  }
  
  inline int speed() {
    return speedCntr;
  }

  inline int accel() {
    return accelCntr;
  }

  void print() {  
    int h1 = digitalRead(h1Pin);
    int h2 = digitalRead(h2Pin);
    int h3 = digitalRead(h3Pin);
    Serial.printf("Hall values: %d, %d, %d\n", h1, h2, h3);
  }
};

// ---------------------------------------------------------------------------------

void hallSensorsTest()
{
  Hall rightHall( 14, 15, 16 );
  Hall leftHall( 18, 19, 20 );
  while( 1 ) {
    Serial.printf( "right ticks/bad_ctr=%d/%d, left ticks/bad_ctr=%d/%d\n",\
                    rightHall.ticks(), rightHall.badState(),\
                    leftHall.ticks(), leftHall.badState() );
  }  
}

// ---------------------------------------------------------------------------------
// ------------------------------------ Motor --------------------------------------
// ---------------------------------------------------------------------------------

class Motor {
  private:
  HardwareSerial* serial;
  int16_t valueLast;

  public:
  Hall hall;
  Motor( HardwareSerial* serialPrm,
         int hallH1Pin,
         int hallH2Pin,
         int hallH3Pin ) :
    serial( serialPrm ),
    valueLast( 0 ),
    hall( hallH1Pin, hallH2Pin, hallH3Pin ) {
    }

  // ---------------------------------------------------------------------------------
  
  inline void torque( int16_t value ) {
    serialWriteFrame( serial, -value );
  }

  // ---------------------------------------------------------------------------------

  void torqueSmooth( int16_t value ) {
    int diff = value - valueLast;
    if( diff > 0 ) {
     valueLast += 2;        
    }
    else if( diff < 0 ) {
      valueLast -= 2;     
    }
    torque( valueLast );  
  }

  // ---------------------------------------------------------------------------------

  void service( int16_t value ) {
    hall.ticks();
    torqueSmooth( value );  
  }  
};

// ---------------------------------------------------------------------------------

class Motors {
  private:
  
  public:
  Motor right;
  Motor left;
  Motors() :
    right( &Serial3, 14, 15, 16 ), // Serial and Hall sensors pins for right motor
    left( &Serial1, 18, 19, 20 ) { // Serial and Hall sensors pins for left motor
  }

  // ---------------------------------------------------------------------------------
  
  inline void reset() {
    right.hall.reset();
    left.hall.reset();  
  }
  
  // ---------------------------------------------------------------------------------
  
  inline void torque( int16_t value ) {
    right.torque( value );
    left.torque( value );
  }
  
  inline void torqueSmooth( int16_t value ) {
    right.torqueSmooth( value );
    left.torqueSmooth( value );
  }

  // ---------------------------------------------------------------------------------

  inline void service( int16_t value ) {
    right.service( value );
    left.service( value );
  }

  // ---------------------------------------------------------------------------------
  
  void windBack( int torquePrm )
  {
    int rightTicks;
    int leftTicks;

    // Continue to apply torque as long as motors are turning.
    do {
      rightTicks = right.hall.ticks();
      leftTicks = left.hall.ticks();
      Serial.printf("Windback torque=%d\n", torquePrm);
      for(int i=0; i<100; i++) {
        torqueSmooth( torquePrm );
        right.hall.ticks();
        left.hall.ticks();
      }
    } while( rightTicks > right.hall.ticks() ||
             leftTicks > left.hall.ticks() );
    torque( 0 );
  }
};

//-------------------------------------------------------------------------

void motorsUpDownTest( int max )
{
  // Loop n times and in between write same value to UART1 multiple times
  Motors motors;
  
  int i=0;
  const uint loopCnt = 250;
  const uint loopDelay = 0;
  for(; i<max; i+=10)
  //for(; i<max; i+=1 )
  {
    Serial.printf("UART1 Sending: %d\n", i);
    Serial.println( "----------------------------" );
    for(int j=0; j<loopCnt; j++) {
      delay(loopDelay);
      motors.torque( i );
      //motors.right.hall.print();      
    }    
  }

  for(; i> -max; i-=10)
  {
    Serial.printf("UART1 Sending: %d\n", i);
    Serial.println( "----------------------------" );
    for(int j=0; j<loopCnt; j++) {
      delay(loopDelay);
      motors.torque( i );
      //motors.right.hall.print(); 
    }    
  }

  for(; i<0; i+=10)
  {
    Serial.printf("UART1 Sending: %d\n", i);
    Serial.println( "----------------------------" );
    for(int j=0; j<loopCnt; j++) {
      delay(loopDelay);
      motors.torque( i );
      //motors.right.hall.print();      
    }    
  }
}

// ---------------------------------------------------------------------------------
// ------------------------------ Workout Profiles ---------------------------------
// ---------------------------------------------------------------------------------

// Workout profiles.
// Specifies 8 bits resistance value for each cable pull distance in cm.

#define W1 50
byte weight_tbl[] = {/*0,   0,   0,   0,   0,   0,   0,   0,*/  0,   0,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,
                       W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1,  W1 };

workout_prf_t weight_prf = { "Weight", 0, 0, 4, 4, sizeof(weight_tbl), weight_tbl };

// ---------------------------------------------------------------------------------

byte spring_tbl[] = {   /*0,   0,   0,   0,   0,   0,   0,   0,   0,   0,*/
                        0,   1,   2,   3,   4,   5,   6,   7,   8,   9,
                       10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
                       20,  21,  22,  23,  24,  25,  26,  27,  28,  29,
                       30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
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

workout_prf_t spring_prf = { "Spring", 0, 0, 4, 4, sizeof(spring_tbl), spring_tbl };

// ---------------------------------------------------------------------------------

byte inv_spring_tbl[] = {/*  0,  0,  0,  0,  0,  0,  0,  0,*/  0,  0,
                          149,148,147,146,145,144,143,142,141,140,
                          139,138,137,136,135,134,133,132,131,130,
                          129,128,127,126,125,124,123,122,121,120,
                          119,118,117,116,115,114,113,112,111,110,
                          109,108,107,106,105,104,103,102,101,100,
                          99,98,97,96,95,94,93,92,91,90,
                          89,88,87,86,85,84,83,82,81,80,
                          79,78,77,76,75,74,73,72,71,70,
                          69,68,67,66,65,64,63,62,61,60,
                          59,58,57,56,55,54,53,52,51,50,
                          49,48,47,46,45,44,43,42,41,40,
                          39,38,37,36,35,34,33,32,31,30,
                          29,28,27,26,25,24,23,22,21,20,
                          19,18,17,16,15,14,13,12,11,10,
                          9,8,7,6,5,4,3,2,1,0 };

workout_prf_t inv_spring_prf = { "Inv-Spring", 0, 0, 4, 4, sizeof(inv_spring_tbl), inv_spring_tbl };

// ---------------------------------------------------------------------------------

byte mtn_tbl[] = {  /* 0,   0,   0,   0,   0,   0,   0,   0,*/   0,   0,
                    50,  52,  54,  56,  58,  60,  62,  64,  66,  68,
                    70,  72,  74,  76,  78,  80,  82,  84,  86,  88,
                    90,  92,  94,  96,  98, 100, 102, 104, 106, 108,
                   110, 112, 114, 116, 118, 120, 122, 124, 126, 128,
                   128, 126, 124, 122, 120, 118, 116, 114, 112, 110,
                   108, 106, 104, 102, 100,  98,  96,  94,  92,  90,
                    88,  86,  84,  82,  80,  78,  76,  74,  72,  70,
                    68,  66,  64,  62,  60,  58,  56,  54,  52,  50 };
       
workout_prf_t mtn_prf = { "Mountain", 0, 0, 4, 4, sizeof(mtn_tbl), mtn_tbl };

// ---------------------------------------------------------------------------------

byte v_tbl[] =   {/*0,   0,   0,   0,   0,   0,   0,   0,*/  0,   0,
                   128, 126, 124, 122, 120, 118, 116, 114, 112, 110,
                   108, 106, 104, 102, 100,  98,  96,  94,  92,  90,
                    88,  86,  84,  82,  80,  78,  76,  74,  72,  70,
                    68,  66,  64,  62,  60,  58,  56,  54,  52,  50,
                    50,  52,  54,  56,  58,  60,  62,  64,  66,  68,
                    70,  72,  74,  76,  78,  80,  82,  84,  86,  88,
                    90,  92,  94,  96,  98, 100, 102, 104, 106, 108,
                   110, 112, 114, 116, 118, 120, 122, 124, 126, 128 };
       
workout_prf_t v_prf = { "V-Shape", 0, 0, 4, 4, sizeof(v_tbl), v_tbl };

// ---------------------------------------------------------------------------------
// ----------------------------------- Cable ---------------------------------------
// ---------------------------------------------------------------------------------

// Geometry
#define TICKS_PER_ROTATION 89.0
#define WHEEL_DIAMETER 12.5
#define DIRECTION_COMP 100

class Cable {
  private:
  Motor* motor;
  workout_prf_t* prf;
  int ticks;
  int distanceRaw;
  int distance;
  int torque;
  direction_t direction;
  int speed;
  int speedPrev;
  int directionComp;

  public:
  Cable( Motor* motorPrm, workout_prf_t* prfPrm ) :
    motor( motorPrm ),
    prf( prfPrm ),
    ticks( 0 ),
    distanceRaw( 0 ),
    distance( 0 ),
    torque( 0 ),
    direction( DIRECTION_PULL ),
    speed( 0 ),
    speedPrev( 0 ),
    directionComp( 0 ) {
  }

  // ---------------------------------------------------------------------------------

  void setPrf( workout_prf_t* prfPrm ) {
    prf = prfPrm;
    direction = DIRECTION_PULL;
    speedPrev = 0; 
  }

  // ---------------------------------------------------------------------------------
  
  int dist() {
    return distance;
  }

  int torq() {
    return torque;
  }

  direction_t dir() {
    return direction;
  }

  // ---------------------------------------------------------------------------------
  
  inline int dirComp( int comp ) {
    if( comp > 0 ){
      if( directionComp < comp ) directionComp++;
    }
    else {
      if( directionComp > 0 ) directionComp--;
    }  
    return directionComp;
  }
  // ---------------------------------------------------------------------------------
  
  void workout() {
    direction = DIRECTION_PULL;
    //speedPrev = 0;
    
    ticks = motor->hall.ticks();
    distanceRaw = (int) ((PI * WHEEL_DIAMETER * ticks) / TICKS_PER_ROTATION);
    distance = constrain( distanceRaw, 0, (prf->len - 1) );
    torque = prf->tbl[ distance ];
      
    int speed = motor->hall.speed();
    if( speed > 0 ) {
      direction = DIRECTION_PULL;
    }
    else if( speed < 0 ) {
      direction = DIRECTION_REL;
    }
    else if( speedPrev > 0 ) {
      direction = DIRECTION_REL;
    }
    else if( speedPrev < 0 ) {
      direction = DIRECTION_PULL;
    }
    speedPrev = speed;
    
    if( direction == DIRECTION_PULL ) {
      torque *= prf->mult_pull;
      if( torque != 0 ) {
        torque += prf->add_pull;
        torque += dirComp( 0 );
      }              
    }
    else {  
      torque *= prf->mult_rel;
      if( torque != 0 ) {
        torque += prf->add_rel;
        torque += dirComp( DIRECTION_COMP );        
      }           
    }
    torque -= speed;
    torque -= motor->hall.accel()/4; //2; 
    
    //if( right_speed <= 0 && right_distance > 20 && right_torque < DIRECTION_COMP) {
    if( distance < 3 && torque < DIRECTION_COMP ) {
      torque = DIRECTION_COMP; //+= 2;  
    }
        
    if( distance <= -50 ) {
      torque = 0;
    }
    
    torque = max( torque, 0 );
    motor->torqueSmooth( torque );
  }
};

// ---------------------------------------------------------------------------------
// ---------------------------------- Machine --------------------------------------
// ---------------------------------------------------------------------------------

class Machine {
  private:
  Motors motors;
  workout_prf_t* prf;
  Cable rightCable;
  Cable leftCable;  
    
  // ---------------------------------------------------------------------------------
  
  public:
  Machine() :
    prf( &weight_prf ),
    rightCable( &motors.right, prf ),
    leftCable( &motors.left, prf ) {
  }

  // ---------------------------------------------------------------------------------
  
  void workout( workout_prf_t* prfPrm ) {
    
    prf = prfPrm;
    rightCable.setPrf( prf );
    leftCable.setPrf( prf );

    int cnt= 0;
    int printCnt = 0;
    while( continueWorkout() )
    {
      rightCable.workout();
      leftCable.workout();

      if( ++printCnt > 12 ) //12
      {
        printCnt=0;
        cnt++;
        
        // Print ticks, distance and torque.

        //Serial.printf("direction=%d\n", rightCable.dir() );
        Serial.printf("cnt=%d, prf=%s, add=%d/%d, mult=%d/%d, ticks=%d/%d, dist=%d/%d, speed=%d/%d, accel=%d/%d, torque=%d/%d\n", \
                     cnt++, prf->name, prf->add_pull, prf->add_rel, prf->mult_pull, prf->mult_rel, motors.right.hall.ticks(), motors.left.hall.ticks(),
                     rightCable.dist(), leftCable.dist(), motors.right.hall.speed(), motors.left.hall.speed(),
                     motors.right.hall.accel()/10, motors.left.hall.accel()/10, rightCable.torq(), leftCable.torq() );
      }
    }
  }

  // ---------------------------------------------------------------------------------

  inline void serviceMotors() {
    motors.service( 150 );      
  }

  // ---------------------------------------------------------------------------------

  void waitForStart()
  {
    // Wait for cables pull.
    while( motors.right.hall.ticks() == 0 && motors.left.hall.ticks() == 0 ) {
      Serial.printf("Waiting for cables pull...\n");
      motors.torque( 0 );  
    }   
  }

  // ---------------------------------------------------------------------------------

  bool continueWorkout()
  {
    if( Serial.available() ) {
      if( Serial.peek() == '\n' || Serial.peek() == '\r') {
        Serial.read();
        return true;
      }
      return false;
    }
    return true;  
  }

  // ---------------------------------------------------------------------------------

  void printCmd( char cmd )
  {
    for( int i=0; i<1000; i++ ) {
      serviceMotors();
      Serial.printf("received %c\n", cmd );  
    }
  }

  // ---------------------------------------------------------------------------------

  void strengthTest()
  {
    static const int convTbl[][2] = { {225, 30}, {300, 35}, {350, 40}, {400, 45}, {450, 50}, {500, 55}, {600, 60}, {750, 65}, {0, 0} };
    motors.windBack( 150 );
    motors.reset();

    while( continueWorkout() && motors.right.hall.ticks() < 50 ) {
      motors.service( 0 );
      Serial.printf("Waiting for cable pull...\n" );
    }

    while( continueWorkout() && motors.right.hall.speed() > 0 ) {
      motors.service( 50 );
      Serial.printf("Stop pulling for strenght test start...\n" );
    }

    int ticks = motors.right.hall.ticks();
    int torque = 150;
    int torqueMax = torque;
    while( continueWorkout() && motors.right.hall.ticks() > 50 && ticks > 50 &&
         (motors.right.hall.ticks() + 50) > ticks )
    {
      Serial.printf("Strength test torque=%d\n", torque );
      for(int i=0; i<100; i++) {
        motors.right.hall.ticks();
        motors.left.hall.ticks();
        motors.right.torque( torque );
        motors.left.torque( 150 );        
      }      
      torqueMax = max( torqueMax, torque );
      torque += 10;
    }

    int poundMax = convTbl[0][1];
    for(int i=0; convTbl[i][0] > 0; i++) {
      if( convTbl[i][0] > torqueMax ) {
        break;
      }
      poundMax = convTbl[i][1];
    }
  
    while( continueWorkout() ) {
      motors.torque( 150 );
      Serial.printf("Strength test done, max torque/pound=%d/%d lb\n", torqueMax, poundMax );
    }
  }
  
  // ---------------------------------------------------------------------------------

  void prfAdjust( int* add, int* mult )
  {
    while( Serial.available() )
    {
      int val = Serial.read();
      switch( val )
      {
        case '+':
          *add += 10;
          break;
            
        case '-':
          *add -= 10;
          break;

        case '*':
          *mult += 1;
          break;
      
        case '/':
          *mult = constrain( *mult-1, 1, *mult );
          break;

        case '0':
          *add = 0;
          *mult = 4;
          break;
      }   
    }        
  }
  
  // ---------------------------------------------------------------------------------

  void main()
  {
    //workout_prf_t* prf = &weight_prf;
    
    waitForStart();
    while(1)
    {
      if( !Serial.available() ) {
        motors.windBack( 150 );
        motors.reset();
        workout( prf );
      }
      
      int input = Serial.read();
      //printCmd( input );
      switch( input )
      {
        case '\n':
        case '\r':
          break;
      
        case 'w':
          prf = &weight_prf;
          break;

        case 's':
          prf = &spring_prf;
          break;

        case 'i':
          prf = &inv_spring_prf;
          break;
        
        case 'm':
          prf = &mtn_prf;
          break;
      
        case 'v':
          prf = &v_prf;
          break;
        
        case 't':
          strengthTest();
          break;
      
        case '+':
          prf->add_pull += 10;
          prf->add_rel += 10;
          break;

        case '-':
          prf->add_pull -= 10;
          prf->add_rel -= 10;
          break;  
      
        case '*':
          prf->mult_pull += 1;
          prf->mult_rel += 1;
          break;

        case '/':
          prf->mult_pull = constrain( prf->mult_pull-1, 1, prf->mult_pull );
          prf->mult_rel = constrain( prf->mult_rel-1, 1, prf->mult_rel );
          break;
      
        case 'p':
          prfAdjust( &prf->add_pull, &prf->mult_pull );
          break;
            
        case 'r':
            prfAdjust( &prf->add_rel, &prf->mult_rel );
            break;

        case '0':
          prf->add_pull = prf->add_rel = 0;
          prf->mult_pull = prf->mult_rel = 4;
          break;
        
        default:
          Serial.printf("received invalid input\n");
          break;
      }    
    }
  }
};

// ---------------------------------------------------------------------------------
// --------------------------------- Main Loop -------------------------------------
// ---------------------------------------------------------------------------------

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

  // Turn on LED. Pin 13 has an LED connected on most Arduino boards.
  int led = 13;
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);  
}

//---------------------------------------------------------------------------

Machine machine;

void loop()
{
  machine.main();
  
  //motor_up_down_test( 200 );
  //hall_sensors_test();
    
  Serial.println( "----------------------------" );
}
