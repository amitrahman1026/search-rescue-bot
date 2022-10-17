#include "serialize.h"
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;
#define PIN5 (1 << 5)
#define PIN6 (1 << 6)
#define PIN11 (1 << 3)
#define PIN10 (1 << 2)

#define COUNTS_PER_REV      180
#define WHEEL_CIRC          22
#define ALEX_LENGTH 30
#define ALEX_BREADTH 30
#define PI                  3.141592654

float AlexCirc = 0.0;
float AlexDiagonal = 0.0;

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variable to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/** ---------------------------- START: GPIO BAREMETAL PROGRAMMING (Completed) ---------------------------------- */
void enablePullups()
{
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch (dir) {
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case LEFT:
      leftReverseTicksTurns++;
      break;
    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }
}

void rightISR()
{
  switch (dir) {
    case FORWARD:
      rightForwardTicks++;
      break;
    case BACKWARD:
      rightReverseTicks++;
      break;
    case LEFT:
      rightForwardTicksTurns++;
      break;
    case RIGHT:
      rightReverseTicksTurns++;
      break;
  }
}

void setupEINT()
{
  EICRA |= 0b00001010;
  EIMSK |= 0b00000011;
}

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}
/** ---------------------------- END: GPIO BAREMETAL PROGRAMMING (Completed) ---------------------------------- */

/**
*/

/** ---------------------------- START: USART BAREMETAL PROGRAMMING (Completed) ---------------------------------- */
char RXBuffer[PACKET_SIZE];
volatile int RXcount = 0;

char TXBuffer[PACKET_SIZE];
// Technically, this TXLen is just to store the variable of serialize
// but I realised that we do not even need it because we always return 
// PACKET_SIZE = 140
volatile int TXLen; 
volatile int TXCounter = 0;

TResult readPacket(TPacket *packet)
{
  if (RXcount == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(RXBuffer, RXcount, packet);
}

void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  TXLen = serialize(TXBuffer, &statusPacket, sizeof(TPacket));
  UCSR0B |= 0b00100000;
}

void sendMessage(const char *message)
{

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  TXLen = serialize(TXBuffer, &messagePacket, sizeof(TPacket));
  UCSR0B |= 0b00100000;
}

void dbprint(char *format, ...) {
  va_list args;
  va_start(args, format);
  vsprintf(TXBuffer, format, args);
  sendMessage(TXBuffer);
}

void sendBadPacket()
{

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  TXLen = serialize(TXBuffer, &badPacket, sizeof(TPacket));
  UCSR0B |= 0b00100000;
}

void sendBadChecksum()
{

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  TXLen = serialize(TXBuffer, &badChecksum, sizeof(TPacket));
  UCSR0B |= 0b00100000;
}

void sendBadCommand()
{

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  TXLen = serialize(TXBuffer, &badCommand, sizeof(TPacket));
  UCSR0B |= 0b00100000;
}

void sendOK()
{

  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  TXLen = serialize(TXBuffer, &okPacket, sizeof(TPacket));
  UCSR0B |= 0b00100000;
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  TXLen = serialize(TXBuffer, &badResponse, sizeof(TPacket));
  UCSR0B |= 0b00100000;
}

void setupUART()    {
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0C = 0b00000110;
  UCSR0A = 0;
}

void startUART()    {
  UCSR0B = 0b10011000; 
}

void setupSerial()  {
  setupUART();
  startUART();
}

/**
 * The ISR will only trigger when there is data to 
 * receive. Even though we do not know whether there
 * is still data to be received (in Arduino Language
 * Serial.available()), we know that the size of the
 * PACKET is 140 because Arduino-pi uses the same
 * serialize packet in serialize.cpp as ALEX.ino. Thus,
 * we just keep track of how many times the RX ISR was
 * triggered. Visit Line 659 for explanation.
 */

ISR(USART_RX_vect)  {
  RXBuffer[RXcount] = UDR0;
  RXcount ++;
}

/**
 * We know that the PACKET_SIZE to be transmitted is 
 * 140. According to the data sheet, UDRE is cleared
 * when we write UDR0 or when we deactivate it. Thus,
 * this ISR will keep on running so we just need to 
 * let the ISR run 140 times to transfer over the 140
 * bytes of data. After that, we can just deactivate
 * it.
 */


ISR(USART_UDRE_vect)    {
  if (TXCounter < PACKET_SIZE)  {
    unsigned char data = TXBuffer[TXCounter];
    UDR0 = data;
    TXCounter += 1;
  } else  {
    TXCounter = 0;
    UCSR0B &= 0b11011111;
  }
}
/** ---------------------------- END: USART BAREMETAL PROGRAMMING (Completed) ---------------------------------- */
/**
*/

/** ---------------------------- START: PWM BAREMETAL PROGRAMMING ---------------------------------- */

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10  // Right reverse pin

int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

void setupMotors()
{
}

void stop() {
  dir = STOP;
  //disable the four pins used to generate pwm
  TCCR0A = 0b00000000;
  TCCR1A = 0b00000000;
  TCCR2A = 0b00000000;
  //set the pwm value to 0
  OCR0A = 0;
  OCR0B = 0;
  OCR1B = 0;
  OCR2A = 0;
}


void forward(float dist, float speed)
{
  if (dist > 0) {
    deltaDist = dist;
  } else {
    deltaDist = 9999999;
  }

  newDist = forwardDist + deltaDist;

  dir = FORWARD;

  int val = pwmVal(speed);
  //have a special function when speed is 75 as Alex's two motors behave differently when power decreases from 100 to 75 
  if (speed == 75.0)  {
    //set the counters to 0
    TCNT0 = 0;
    TCNT2 = 0;
    //enable the pin 5 to be set or cleared on compare matches
    TCCR0A = 0b00100001;
    //enable the pin 11 to be set or cleared on compare matches
    TCCR2A = 0b10000001;
    //set pre-scaler to 64
    TCCR0B = 0b00000011;
    TCCR2B = 0b00000011;
    //set the OCR0B to the val bewteen 0 and 255 to achieve a duty cycle same as the speed
    OCR0B = val;
    //set the OCR2A to the val*0.84 (after trial and error and we chose the value for Alex to go straight)bewteen 0 and 255 to achieve a duty cycle same as the speed
    OCR2A = val * 0.84;
    //set pin5 and pin11 as the output pins
    DDRD |= PIN5;
    DDRB |= PIN11;
  } else  {
    //set the counters to 0
    TCNT0 = 0;
    TCNT2 = 0;
    //enable the pin 5 to be set or cleared on compare matches
    TCCR0A = 0b00100001;
    //enable the pin 11 to be set or cleared on compare matches
    TCCR2A = 0b10000001;
    //set pre-scaler to 64
    TCCR0B = 0b00000011;
    TCCR2B = 0b00000011;
    //set the OCR0B to the val bewteen 0 and 255 to achieve a duty cycle same as the speed
    OCR0B = val;
    //set the OCR2A to the val bewteen 0 and 255 to achieve a duty cycle same as the speed
    OCR2A = val;
    //set pin5 and pin11 as the output pins
    DDRD |= PIN5;
    DDRB |= PIN11;
  }
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  // Code to rell us how far to move
  if (dist == 0) {
    deltaDist = 9999999;
  } else {
    deltaDist = dist;
  }

  newDist = reverseDist + deltaDist;

  dir = BACKWARD;

  int val = pwmVal(speed);

  if (speed == 75.0)  {
    //set the counters to 0
    TCNT0 = 0;
    TCNT1 = 0;
    //enable the pin 6 to be set or cleared on compare matches
    TCCR0A = 0b10000001;
    //enable the pin 10 to be set or cleared on compare matches
    TCCR1A = 0b00100001;
    //set pre-scaler to 64
    TCCR0B = 0b00000011;
    TCCR1B = 0b00000011;
    //set the OCR0A to the val bewteen 0 and 255 to achieve a duty cycle same as the speed
    OCR0A = val;
    //set the OCR1B to the val*0.85 (after trial and error and we chose the value for Alex to go straight) bewteen 0 and 255 to achieve a duty cycle same as the speed
    OCR1B = val * 0.85;
    //set pin6 and pin10 as the output pins
    DDRD |= PIN6;
    DDRB |= PIN10;
  } else {
    TCNT0 = 0;
    TCNT1 = 0;
    TCCR0A = 0b10000001;
    TCCR1A = 0b00100001;
    TCCR0B = 0b00000011;
    TCCR1B = 0b00000011;
    OCR0A = val;
    OCR1B = val;
    DDRD |= PIN6;
    DDRB |= PIN10;
  }

}

// Function to estimate the number of wheel ticks needed to turn an angle
unsigned long computeDeltaTicks(float ang)
{
  // We will assume that angular distance moved = linerar distance moved in one wheel
  // revolution. This is (probably) incorrect but simplifies calculation.
  // # of wheel revs to make one full 360 turn is AlexCirc / WHEEL_CIRC
  // This is for 360 degrees. For ang degrees it will be (ang * AlexCirc) / (360.0 * WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV

  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  //after trial and error we found the ang need to be divided by 5.1 in order for Alex to turn the same angle as what we entered)
  ang /= 5.1;

  int val = pwmVal(speed);
  if (ang == 0) {
    deltaTicks = 9999999;
  } else  {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = leftReverseTicksTurns + deltaTicks;
  
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  //set the counters to 0
  TCNT0 = 0;
  TCNT2 = 0;
  //enable the pin 6 to be set or cleared on compare matches
  TCCR0A = 0b10000001;
  //enable the pin 11 to be set or cleared on compare matches
  TCCR2A = 0b10000001;
  //set pre-scaler to 64
  TCCR0B = 0b00000011;
  TCCR2B = 0b00000011;
  //set the OCR0A to the val bewteen 0 and 255 to achieve a duty cycle same as the speed
  OCR0A = val;
  //set the OCR2A to the val bewteen 0 and 255 to achieve a duty cycle same as the speed
  OCR2A = val;
  //set pin6 and pin11 as the output pins
  DDRD |= PIN6;
  DDRB |= PIN11;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  //after trial and error we found the ang need to be divided by 5.4 in order for Alex to turn the same angle as what we entered)
  ang /= 5.4;
  int val = pwmVal(speed);

  if (ang == 0) {
    deltaTicks = 9999999;
  } else  {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = rightReverseTicksTurns + deltaTicks;

  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  //set the counters to 0
  TCNT0 = 0;
  TCNT1 = 0;
  //enable the pin 5 to be set or cleared on compare matches
  TCCR0A = 0b00100001;
  //enable the pin 10 to be set or cleared on compare matches
  TCCR1A = 0b00100001;
  //set pre-scaler to 64
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
  //set the OCR0B to the val bewteen 0 and 255 to achieve a duty cycle same as the speed
  OCR0B = val;
  //set the OCR1B to the val bewteen 0 and 255 to achieve a duty cycle same as the speed
  OCR1B = val;
  //set pin5 and pin10 as the output pins
  DDRD |= PIN5;
  DDRB |= PIN10;
}



/*
   Alex's setup and run codes
*/

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);

    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {

        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup() {
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;

  cli();
  setupEINT();
  setupSerial();
  setupMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  TPacket recvPacket; 

  /**
   * It took me some time to figure out that if we do
   * not set the conditional statement, the incomplete
   * packet will be deserialized and it will send back
   * to the PI a BadPacket.
   * 
   * Therefore, I will let the readPacket run when the
   * PACKET is fully sent over from the Arduino 
   * (i.e. RXCount == 140) so that I am able to accurately
   * read the packet.
   */

  if (RXcount == PACKET_SIZE) {
    TResult result = readPacket(&recvPacket);
    if (result == PACKET_OK) {
      handlePacket(&recvPacket);
      RXcount = 0;
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();

    }
    else if (result == PACKET_CHECKSUM_BAD)
    {
      sendBadChecksum();
    }
  }

  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD)
    {
      if (reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0) {
    if (dir ==  LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else  {
      if (dir ==  RIGHT)  {
        if (rightReverseTicksTurns >= targetTicks) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }
      else  {
        if (dir == STOP)  {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }
    }
  }
}
