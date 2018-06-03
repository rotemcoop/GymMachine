// Teensy UART Tx to be connected to light blue next to black wire on Hoverboard

// Pin 13 has an LED connected on most Arduino boards.
int led = 13;

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

void hall_sensors_test()
{
  enum {
    HALL_100 = 0b100,
    HALL_110 = 0b110,
    HALL_010 = 0b010,
    HALL_011 = 0b011,
    HALL_001 = 0b001,
    HALL_101 = 0b101
  };

  // LUT for hal sensors transitions - [prev_state][state].
  // +1 -> CW tick
  // -1 -> CCW tick
  // 00 -> no transition
  // NA -> invalid transition
  // There are 88-89 ticks per revolution
  #define NA 99
  signed char hall_tbl[8][8] = { NA, NA, NA, NA, NA, NA, NA, NA,
                                 NA, 00, NA, -1, NA, +1, NA, NA,
                                 NA, NA, 00, +1, NA, NA, -1, NA,
                                 NA, +1, -1, 00, NA, NA, NA, NA,
                                 NA, NA, NA, NA, 00, -1, +1, NA,
                                 NA, -1, NA, NA, +1, 00, NA, NA,
                                 NA, NA, +1, NA, -1, NA, 00, NA,
                                 NA, NA, NA, NA, NA, NA, NA, NA };
  
  int ticks = 0;
  static int h1, h1_prev;
  static int h2, h2_prev;
  static int h3, h3_prev;
  
  h1_prev = h1 = digitalRead(14);
  h2_prev = h2 = digitalRead(15);
  h3_prev = h3 = digitalRead(16);
    
  uint hall_state = 0;
  uint hall_state_prev = 0;
  uint bad_state_cntr = 0;
    
  // Spin util first valid transition
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
    
  // Spin and calulate hall ticks
  while( 1 )
  {
    // Get hall sensors state
    h1 = digitalRead(14);
    h2 = digitalRead(15);
    h3 = digitalRead(16);
    hall_state = h1 << 2 | h2 << 1 | h3;

    // Get hall tick based on current and previous states.
    // Zero out invalid transitions.
    int tick = hall_tbl[hall_state_prev][hall_state];
    if( tick == NA ) {
      bad_state_cntr++;
      tick = 0;
    }
    ticks += tick;

    hall_state_prev = hall_state;
    
    Serial.printf("ticks=%d, bad_state_cntr=%d\n", ticks, bad_state_cntr);
    //Serial.printf("hall state: %d%d%d\n", h1,h2,h3);
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
