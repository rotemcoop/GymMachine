// ---------------------------------------------------------------------------------
// Wiring
// Teensy UART Tx to be connected to light blue next to black wire on Hoverboard
// ---------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------
// --------------------------------- Golbal Data -----------------------------------
// ---------------------------------------------------------------------------------

// Pin 13 has an LED connected on most Arduino boards.
int led = 13;



// ---------------------------------------------------------------------------------
// ----------------------------------- Methods -------------------------------------
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

  pinMode(17, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
}

//---------------------------------------------------------------------------

const uint8_t    GYRO_FRAME_LENGTH = 6;
const uint16_t   GYRO_FRAME_START = 256;
const uint8_t    GYRO_CONTACT_CLOSED_BYTE = 85;

void serial_write_frame(HardwareSerial serial, int16_t value)
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
    serial.write9bit(uart_frame[i]); 
  } 
}  

//-------------------------------------------------------------------------

// Connect Serial1 Tx to Serial3 Rx
// Connect Serial1 Rx to Serial3 Tx

void uart1_to_uart3_test( uint16_t value )
{
  // Write a frame out of UART1
  serial_write_frame( Serial1, value );

  delay (10);

  // Read fram from UART2 and print via USB
  int incomingByte;
  while (Serial3.available() > 0) {
    incomingByte = Serial3.read();
    Serial.print("UART received: ");
    Serial.println(incomingByte, DEC);
  }
}

//-------------------------------------------------------------------------

void print_hall_sensors()
{  
  int h1 = digitalRead(14);
  int h2 = digitalRead(15);
  int h3 = digitalRead(16);
  Serial.printf("Hall values: %d, %d, %d\n", h1, h2, h3);
}

//-------------------------------------------------------------------------

void motor_up_down_test( int max )
{
  // Loop n times and in between write same value to UART1 multiple times
  
  int i=0;
  const uint loop_cnt = 250;
  const uint loop_delay = 0;
  for(; i<max; i+=10)
  {
    Serial.printf("UART1 Sending: %d\n", i);
    Serial.println( "----------------------------" );
    for(int j=0; j<loop_cnt; j++) {
      delay(loop_delay);
      serial_write_frame( Serial1, i );
      print_hall_sensors();      
    }    
  }

  for(; i> -max; i-=10)
  {
    Serial.printf("UART1 Sending: %d\n", i);
    Serial.println( "----------------------------" );
    for(int j=0; j<loop_cnt; j++) {
      delay(loop_delay);
      serial_write_frame( Serial1, i );
      print_hall_sensors(); 
    }    
  }

  for(; i<0; i+=10)
  {
    Serial.printf("UART1 Sending: %d\n", i);
    Serial.println( "----------------------------" );
    for(int j=0; j<loop_cnt; j++) {
      delay(loop_delay);
      serial_write_frame( Serial1, i );
      print_hall_sensors();      
    }    
  }
}

//-------------------------------------------------------------------------

// Workout profile table.
// Holding 8 bits resistance value for each cable pull distance in cm.
byte p_table[200];

// LUT ( hall_tbl[prev_state][state] ) for mapping hall sensors transitions to ticks.
// +1 -> CW tick
// -1 -> CCW tick
// 00 -> no transition
// NA -> invalid transition
// There are 88-90 ticks per revolution
#define TICKS_PER_ROTATION 89
#define WHEEL_DIAMETER 12.5 
#define NA 99
signed char hall_tbl[8][8] = { NA, NA, NA, NA, NA, NA, NA, NA,
                               NA, 00, NA, -1, NA, +1, NA, NA,
                               NA, NA, 00, +1, NA, NA, -1, NA,
                               NA, +1, -1, 00, NA, NA, NA, NA,
                               NA, NA, NA, NA, 00, -1, +1, NA,
                               NA, -1, NA, NA, +1, 00, NA, NA,
                               NA, NA, +1, NA, -1, NA, 00, NA,
                               NA, NA, NA, NA, NA, NA, NA, NA };
                               
void hall_sensors_test()
{
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

  int ticks = 0;
  static int h1, h1_prev;
  static int h2, h2_prev;
  static int h3, h3_prev;
  
  uint hall_state = 0;
  uint hall_state_prev = 0;
  uint bad_state_cntr = 0;
    
  // Loop until detecting first valid transition
  while( hall_state == hall_state_prev ||
         hall_tbl[hall_state_prev][hall_state] == NA )
  {
    h1 = digitalRead(14);
    h2 = digitalRead(15);
    h3 = digitalRead(16);
    hall_state_prev = hall_state;
    hall_state = h1 << 2 | h2 << 1 | h3;
    Serial.printf("hall_state NA\n");
  }

  // Loop and calulate hall ticks
  while( 1 )
  {
    //int i = 0;
    
    // Get hall sensors state
    h1 = digitalRead(14);
    h2 = digitalRead(15);
    h3 = digitalRead(16);
    hall_state = h1 << 2 | h2 << 1 | h3;

    // Map hall sensors transition to rotation ticks (0, +1, -1)
    // Detect invalid transitions.
    int tick = hall_tbl[hall_state_prev][hall_state];
    if( tick == NA ) {
      bad_state_cntr++;
      tick = 0;
    }
    ticks += tick;
    uint rotations = (PI * WHEEL_DIAMETER * ticks) / TICKS_PER_ROTATION;
    hall_state_prev = hall_state;
    
    // Print out hall sensor ticks
    Serial.printf("ticks=%d, rotation=%u, bad_state_cntr=%d\n", ticks, rotations, bad_state_cntr);
    //Serial.printf("hall state: %d%d%d\n", h1,h2,h3);
    //serial_write_frame( Serial1, 100 );
  }
}

//-------------------------------------------------------------------------

void loop()
{
  // Blink the LED
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(500);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(2000);               // wait for a second
  //motor_up_down_test( 1 );
  serial_write_frame( Serial1, 0 );
  //print_hall_sensors();
  hall_sensors_test();
  Serial.println( "----------------------------" );

  //int incomingByte;
    //while (Serial1.available() > 0) {
    //incomingByte = Serial1.read();
    //Serial.print("UART received: ");
    //Serial.println(incomingByte, DEC);
}
