/*
 * irdroino_serial
 * https://github.com/bome/irdroino
 * (c) 2019 by Bome Software GmbH & Co. KG
 * 
 * For Arduino with the Irdroino shield.
 * 
 * Functions:
 * 1) LEARN: capture commands via the IR detector, parse them, and report them
 *      as an ASCII command to the serial port.
 * 1) SEND: take such ASCII commands via the serial port and send them as IR signals.
 * 3) MISC: the two shield buttons send ASCII commands via serial, and you can turn
 *      on/off the LEDs via Serial port commands.
 * 
 * Main usage is in conjunction with Bome MIDI Translator Pro:
 * https://www.bome.com/products/miditranslator
 * and the BomeBox:
 * https://www.bome.com/products/bomebox
 * For interfacing IR with MIDI (and HID) devices via USB, MIDI-DIN,
 * Ethernet and WiFi. Plus, extensive mapping and macros possible.
 * 
 * Usage:
 * At start-up, the sketch reports "BOME SERIAL IR READY" to the serial port.
 * 
 * 
 * LEARN
 * 1) connect Arduino to computer
 * 2) open serial port with 115200 baud
 * 3) direct an IR remote at the IR detector
 * 4) you will see all detected IR commands, e.g.:
 *    NEC1 133,118,48
 *    NEC1 REPEAT
 *    RAW JVC LEN:16 20F0
 *    RAW PANASONIC ADDR:5362 9164AD03
 * Note: The "blue" LED will light up when an IR signal is detected.
 * 
 * 
 * SEND
 * Via a serial port, you can send the exact same commands
 * which you have captured in the via LEARN to the Arduino
 * to send them via IR. A command is terminated (and executed)
 * upon a new line (\n or \r).
 * 
 * You can also send Pronto codes by prefixing the command
 * with PRONTO. The maximum length of the PRONTO command 
 * (including the command "PRONTO") must not exceed 1024 chars.
 * Example (to be sent all on one line):
 * PRONTO 0000 006C 0022 0002 015B 00AD 0016 0016 0016 0016 
 *   0016 0041 0016 0016 0016 0016 0016 0016 0016 0016 0016
 *   0016 0016 0041 0016 0041 0016 0016 0016 0041 0016 0041
 *   0016 0041 0016 0041 0016 0041 0016 0016 0016 0016 0016
 *   0016 0016 0041 0016 0016 0016 0016 0016 0016 0016 0016
 *   0016 0041 0016 0041 0016 0041 0016 0016 0016 0041 0016
 *   0041 0016 0041 0016 0041 0016 05F7 015B 0057 0016 0E6C
 * 
 * The "orange" LED will light up when a command is sent.
 * 
 * 
 * MISC
 * Pressing the buttons on the irdroino shield will send
 * "BUTTON1" ot "BUTTON2" to the serial port.
 * 
 * You can control the LED's with the following commands:
 * "LED1 ON" / "LED1 OFF"
 * "LED2 ON" / "LED2 OFF"
 * 
 * 
 * ERROR HANDLING
 * Any error is signaled via serial port with an error string 
 * which always starts with "ERROR".
 * Successful command execution will not cause a success message.
 * 
 * 
 * INSTALLATION
 * Requires the patched IRremote library from:
 * https://github.com/bome/Arduino-IRremote
 * 
 * Install it in the "library" folder of your Arduino installation 
 * on the computer.
 */

// If the following is defined (default), sending PRONTO is done 
// via the IRremote lib, which requires the patched/fixed 
// Arduino-IRremote library from:
// https://github.com/bome/Arduino-IRremote
//
// Otherwise, if not defined, the code from pronto_usb_ir_blaster.ino 
// is used (included at bottom), and you need to make a jumper wire 
// connection between pin 9 and pin 3.
#define USE_IRREMOTE_PRONTO

#include <IRremote.h>
#include <avr/pgmspace.h>

#ifdef USE_IRREMOTE_PRONTO
#include <irPronto.cpp>
#else
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#endif //USE_IRREMOTE_PRONTO

// Use faster 115200 baud rate
#define BAUD_RATE  (115200)

// Uncomment this for using default Arduino 9600 baud rate.
//#define BAUD_RATE  (9600)

const int RECV_PIN = 2;
const int SEND_PIN = 3;
const int buttonPin1 = 4;     // irdroino button 1
const int buttonPin2 = 5;     // irdroino button 2
const int blueLedPin = 6;     // Blue LED
const int orangeLedPin = 7;   // Orange LED

const int SERIAL_INPUT_BUFFER_SIZE = 1024 * 1;

const int decode_type_t_count = LEGO_PF + 1;

IRrecv irrecv(RECV_PIN);
IRsend irsend;

void setup()
{
  Serial.begin(BAUD_RATE);
  while (!Serial);
  // Initialize the Orange LED pin as an output
  pinMode(orangeLedPin, OUTPUT);      
  // Initialize the "Send" push-button pin as an input
  pinMode(buttonPin1, INPUT_PULLUP);  
  // Initialize the Orange LED pin as an output
  pinMode(blueLedPin, OUTPUT);      
  // Initialize the "Send" push-button pin as an input
  pinMode(buttonPin2, INPUT_PULLUP);
  // Start the receiver
  irrecv.enableIRIn();
  // signal start
  Serial.println(F("BOME SERIAL IR READY"));
}

//wrap plain strings
#define CF(s)  s

const char* encoding2string(decode_type_t type)
{
  // use PROGMEM to save 122 bytes heap
  #define ENCODING_STRING_MAX_LEN (11)
  static char ret_val[ENCODING_STRING_MAX_LEN + 1];

  #define RETURN(string) \
  {\
    static const char s[] PROGMEM = string; \
    return strncpy_P(ret_val, s, ENCODING_STRING_MAX_LEN); \
  }

  // make sure that the resulting string is zero-terminated
  ret_val[ENCODING_STRING_MAX_LEN] = 0;
  switch (type) {
    case RC5:          RETURN("RC5");
    case RC6:          RETURN("RC6");
    case NEC:          RETURN("NEC");
    case SONY:         RETURN("SONY");
    case PANASONIC:    RETURN("PANASONIC");
    case JVC:          RETURN("JVC");
    case SAMSUNG:      RETURN("SAMSUNG");
    case WHYNTER:      RETURN("WHYNTER");
    case AIWA_RC_T501: RETURN("AIWA");
    case LG:           RETURN("LG");
    case SANYO:        RETURN("SANYO");
    case MITSUBISHI:   RETURN("MITSUBISHI");
    case DISH:         RETURN("DISH");
    case SHARP:        RETURN("SHARP");
    case DENON:        RETURN("DENON");
    case PRONTO:       RETURN("PRONTO");
    case LEGO_PF:      RETURN("LEGO");
  }
  return NULL;
  #undef RETURN
}

int encoding2defaultBits(decode_type_t type)
{
  switch (type) {
    //case RC5:          return 12;
    //case RC6:          return XX;
    case NEC:          return 32;
    //case SONY:         return XX;
    //case PANASONIC:    return XX;
    case JVC:          return 16;
    //case SAMSUNG:      return XX;
    //case WHYNTER:      return XX;
    //case AIWA_RC_T501: return XX;
    //case LG:           return XX;
    //case SANYO:        return XX;
    //case MITSUBISHI:   return XX;
    //case DISH:         return XX;
    //case SHARP:        return XX;
    //case DENON:        return XX;
    //case PRONTO:       return XX;
    //case LEGO_PF:      return XX;
  }
  // max default
  return 32;
}

// from: https://stackoverflow.com/questions/2602823/in-c-c-whats-the-simplest-way-to-reverse-the-order-of-bits-in-a-byte
unsigned char reverseLowByte(unsigned char b)
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void skipWhiteSpace(const char** received)
{
  // skip white space
  while (*(*received) != 0 && *(*received) <= 32)
  {
    (*received)++;
  }
}

// Checks if "*received" starts with the given command.
// If yes, advances *received to the next non-white space
// after the command and return true.
// Otherwise, or if command is empty, return false.
bool startsWithCommand(const char** received, char* command)
{
  if (command == NULL || command[0] == 0)
  {
    return false;
  }
  skipWhiteSpace(received);
  int len = strlen(command);
  if (strncmp(*received, command, len) == 0)
  {
    (*received) += len;
    // NB: don't skip white space here.
    return true;
  }
  return false;
}

// return true if handled
bool printNEC1(decode_results *results)
{
  if (results->value == REPEAT)
  {
    Serial.println(F("NEC1 REPEAT"));
    return true;
  }
  unsigned char device = reverseLowByte((unsigned char)((results->value >> 24) & 0xFF));
  unsigned char subdevice = reverseLowByte((unsigned char)((results->value >> 16) & 0xFF));
  unsigned char function = reverseLowByte((unsigned char)((results->value >> 8) & 0xFF));
  unsigned char chk = reverseLowByte((unsigned char)(results->value & 0xFF));
  
  if (255 - function != chk)
  {
    return false;
  }
  Serial.print(F("NEC1 "));
  Serial.print(device, DEC);
  Serial.print(F(","));
  if (255 - device != subdevice)
  {
    // 3-value command
    Serial.print(subdevice, DEC);
    Serial.print(F(","));
  }
  Serial.println(function, DEC);
  return true;
}

void printCode(decode_results *results)
{
  const char* typeString = encoding2string(results->decode_type);
  if (typeString == NULL)
  {
    Serial.println(F("ERROR: RECV PARSER"));
    return;
  }

  if (results->overflow)
  {
    Serial.println(F("ERROR: RECV OVERFLOW"));
    return;
  }

  // special handling for NEC protocol
  if (results->decode_type == NEC && printNEC1(results))
  {
    return;
  }

  Serial.print(F("RAW "));
  Serial.print(typeString);
  Serial.print(F(" "));

  if (results->value == REPEAT)
  {
    Serial.println(F("REPEAT"));
    return;
  }

  if (results->decode_type == PANASONIC)
  {
    Serial.print(F("ADDR:"));
    Serial.print(results->address, HEX);
    Serial.print(F(" "));
  }
  if (results->bits != encoding2defaultBits(results->decode_type))
  {
    Serial.print(F("LEN:"));
    Serial.print(results->bits, DEC);
    Serial.print(F(" "));
  }
  Serial.println(results->value, HEX);
}

unsigned long parseValue(const char** value, int base)
{
  skipWhiteSpace(value);
  char* p = *value;
  unsigned long ret = strtol(*value, &p, base);
  if (*value == p)
  {
    Serial.print(F("ERROR: number: "));
    Serial.println(*value);
    // parse error
    return 0;
  }
  *value = p;
  return ret;
}

void handleProntoCommand(const char* command)
{
  digitalWrite(orangeLedPin, HIGH);
  if (!irsend.sendPronto((char*)command, PRONTO_ONCE, PRONTO_FALLBACK))
  {
    Serial.println(F("ERROR PRONTO"));
  }
  digitalWrite(orangeLedPin, LOW);
}

void sendRawCommand(decode_type_t type,
                    unsigned long value,
                    int bits = 32,
                    bool repeat = 0,
                    unsigned int address = 0)
{
  digitalWrite(orangeLedPin, HIGH);
  // now send
  switch (type)
  {
  case RC5: // fall through
  case RC6:
  {
    static int toggle = 0; // The RC5/6 toggle state
    if (!repeat)
    {
      // Flip the toggle bit for a new button press
      toggle = 1 - toggle;
    }
    // Put the toggle bit into the code to send
    value = value & ~(1 << (bits - 1));
    value = value | (toggle << (bits - 1));
    if (type == RC5)
    {
      irsend.sendRC5(value, bits);
    } 
    else
    {
      irsend.sendRC6(value, bits);
    }
    break;
  } 
  case NEC:
  {
    if (repeat)
    {
      irsend.sendNEC(REPEAT, bits);
    } 
    else
    {
      irsend.sendNEC(value, bits);
    }
    break;
  } 
  case SONY:
  {
    irsend.sendSony(value, bits);
    break;
  } 
  case PANASONIC:
  {
    irsend.sendPanasonic(address, value);
    break;
  }
  case JVC:
  {
    irsend.sendJVC(value, bits, false);
    break;
  }
  case SAMSUNG:
  {
    irsend.sendSAMSUNG(value, bits);
    break;
  }
  case WHYNTER:
  {
    irsend.sendWhynter(value, bits);
    break;
  }
  case AIWA_RC_T501:
  {
    irsend.sendAiwaRCT501(value);
    break;
  }
  case LG:
  {
    irsend.sendLG(value, bits);
    break;
  }
#ifdef NOT_YET_IMPLEMENTED
  case SANYO:
  {
    irsend.sendSanyo(value, bits);
    break;
  }
  case MITSUBISHI:
  {
    irsend.sendMitsubishi(value, bits);
    break;
  }
#endif //NOT_YET_IMPLEMENTED
  case DISH:
  {
    irsend.sendDISH(value, bits);
    break;
  }
  case SHARP:
  {
    irsend.sendSharpRaw(value, bits);
    break;
  }
  case DENON:
  {
    irsend.sendDenon(value, bits);
    break;
  }
  case LEGO_PF:
  {
    irsend.sendLegoPowerFunctions(value);
    break;
  }
  default:
  {
    digitalWrite(orangeLedPin, LOW);
    Serial.println(F("ERROR: NOT IMPLEMENTED"));
    break;
  }
  } // switch
  digitalWrite(orangeLedPin, LOW);
}


void handleRawCommand(decode_type_t type, const char* command)
{
  unsigned int address = 0;
  unsigned long value = 0;
  int bits = 32;
  bool repeat = false;

  if (startsWithCommand(&command, CF("ADDR:")))
  {
    address = parseValue(&command, 16);
    if (address == 0)
    {
      Serial.println(F("ERROR: INVALID ADDRESS"));
      return;
    }
  }
  if (startsWithCommand(&command, CF("LEN:")))
  {
    bits = parseValue(&command, 10);
    if (bits == 0)
    {
      Serial.println(F("ERROR: INVALID LEN"));
      return;
    }
  }
  if (startsWithCommand(&command, CF("REPEAT")))
  {
    repeat = true;
  }
  else
  { 
    value = parseValue(&command, 16);
    if (value == 0)
    {
      Serial.println(F("ERROR: INVALID VALUE"));
      return;
    }
  }
  sendRawCommand(type, value, bits, repeat, address);
}

// non-raw commands. Currently only NEC1 and NEC2:
// e.g. NEC1 133,118,48
// or: NEC1 REPEAT
// return true if the command was parsed (i.e. it starts with NEC1 or NEC2)
bool handleCommand(const char* command)
{
  // NEC1 device,[subdevice,]function
  // NEC1 REPEAT
  if (command[0] == 'N'
   && command[1] == 'E' 
   && command[2] == 'C' 
   && (command[3] == '1' || command[3] == '2'))
  {
    bool isNEC1 = (command[3] == '1');
    command += 4;
    skipWhiteSpace(&command);

    if (command[0] == 'R'
      && command[1] == 'E')
    {
      // repeat
      sendRawCommand(NEC, REPEAT, 32, true);
      return true;
    }

    unsigned long device = parseValue(&command, 10);
    skipWhiteSpace(&command);
    if (command[0] != ',')
    {
      Serial.println(F("ERROR: SYNTAX NEC1"));
      // return true even on error!
      return true;
    }
    command++;
    unsigned long subdevice = 255 - device;
    unsigned long function = parseValue(&command, 10);
    skipWhiteSpace(&command);
    if (command[0] == ',')
    {
      // command with subdevice
      command++;
      subdevice = function;
      function = parseValue(&command, 10);
    }
    // sanity test
    if (device > 255 || subdevice > 255 || function > 255)
    {
      Serial.println(F("ERROR: NEC1 VALUE RANGE"));
      return;
    }
    device = reverseLowByte((unsigned char)device);
    subdevice = reverseLowByte((unsigned char)subdevice);
    function = reverseLowByte((unsigned char)function);
    unsigned long value = (device << 24)
                        | (subdevice << 16)
                        | (function << 8)
                        | (255 - function);
    //Serial.println(value, HEX);
    sendRawCommand(NEC, value, 32);
    return true;
  }
  return false;
}

void handleProntoCommand2(const char* received);

void handleReceivedCommand(const char* received)
{
  bool ok = false;

  if (startsWithCommand(&received, encoding2string(PRONTO)))
  {
#ifdef USE_IRREMOTE_PRONTO
    handleProntoCommand(received);
#else
    handleProntoCommand2(received);
#endif //USE_IRREMOTE_PRONTO
    ok = true;
  }
  
  if (!ok
      && received[0] == 'R'
      && received[1] == 'A'
      && received[2] == 'W')
  {
    received += 3;
    skipWhiteSpace(&received);

    // parse IR prefixes
    for (int t = 0; t < decode_type_t_count; t++)
    {
      if (startsWithCommand(&received, encoding2string((decode_type_t)t)))
      {
        handleRawCommand((decode_type_t)t, received);
        ok = true;
        break;
      }
    }
  }

  if (!ok)
  {
    ok = handleCommand(received);
  }

  if (!ok)
  {
    if (startsWithCommand(&received, CF("LED1 ON")))
    {
      digitalWrite(blueLedPin, HIGH);
      ok = true;
    }
    else if (startsWithCommand(&received, CF("LED1 OFF")))
    {
      digitalWrite(blueLedPin, LOW);
      ok = true;
    }
    else if (startsWithCommand(&received, CF("LED2 ON")))
    {
      digitalWrite(orangeLedPin, HIGH);
      ok = true;
    }
    else if (startsWithCommand(&received, CF("LED2 OFF")))
    {
      digitalWrite(orangeLedPin, LOW);
      ok = true;
    }
  }
  
  if (!ok)
  {
    Serial.println(F("ERROR: SYNTAX"));
  }

}

void loop()
{
  static uint8_t buttonState1 = 0;        
  static uint8_t buttonState2 = 0;
  static decode_results results;
  
  int newButtonState1 = digitalRead(buttonPin1);
  int newButtonState2 = digitalRead(buttonPin2);

  if (buttonState1 == HIGH && newButtonState1 == LOW) {
    Serial.println(F("BUTTON1"));
    irrecv.enableIRIn(); // Re-enable receiver
  }
  buttonState1 = newButtonState1;

  if (buttonState2 == HIGH && newButtonState2 == LOW) {
    Serial.println(F("BUTTON2"));
    irrecv.enableIRIn(); // Re-enable receiver
  }
  buttonState2 = newButtonState2;

  // received IR signal?
  if (irrecv.decode(&results)) {
    digitalWrite(blueLedPin, HIGH);
    printCode(&results);
    delay(200);
    irrecv.resume(); // resume receiver
    digitalWrite(blueLedPin, LOW);
  }

  // received Serial command?
  if (Serial.available() > 0)
  {
    static char input[SERIAL_INPUT_BUFFER_SIZE];
    static uint16_t readCharCount = 0;
    char c = Serial.read();

    if (c == '\r' || c == '\n')
    {
      if (readCharCount < SERIAL_INPUT_BUFFER_SIZE - 1 && readCharCount > 0)
      {
        input[readCharCount] = '\0';
        handleReceivedCommand(input);
        irrecv.enableIRIn(); // Re-enable receiver
      }
      readCharCount = 0;
    }
    else
    {
      if (readCharCount < SERIAL_INPUT_BUFFER_SIZE - 1)
      {
        input[readCharCount] = c;
      }
      else if (readCharCount == SERIAL_INPUT_BUFFER_SIZE - 1)
      {
        Serial.println(F("ERROR: INPUT BUFFER OVERFLOW"));
      }
      readCharCount++;
    }
  }
}

#ifndef USE_IRREMOTE_PRONTO

// SEND PRONTO implementation from pronto_usb_ir_blaster.ino

#define IR_PORT PORTB
#define IR_PIN PINB
#define IR_DDR DDRB
#define IR_BV _BV(1)
#define IR_OCR OCR1A
#define IR_TCCRnA TCCR1A
#define IR_TCCRnB TCCR1B
#define IR_TCNTn TCNT1
#define IR_TIFRn TIFR1
#define IR_TIMSKn TIMSK1
#define IR_TOIEn TOIE1
#define IR_ICRn ICR1
#define IR_OCRn OCR1A
#define IR_COMn0 COM1A0
#define IR_COMn1 COM1A1
#define PRONTO_IR_SOURCE 0 // Pronto code byte 0
#define PRONTO_FREQ_CODE 1 // Pronto code byte 1
#define PRONTO_SEQUENCE1_LENGTH 2 // Pronto code byte 2
#define PRONTO_SEQUENCE2_LENGTH 3 // Pronto code byte 3
#define PRONTO_CODE_START 4 // Pronto code byte 4

static const uint16_t *ir_code = NULL;
static uint16_t ir_cycle_count = 0;
static uint32_t ir_total_cycle_count = 0;
static uint8_t ir_seq_index = 0;
static uint8_t ir_led_state = 0;

void ir_on()
{
  IR_TCCRnA |= (1<<IR_COMn1) + (1<<IR_COMn0);
  ir_led_state = 1;
}

void ir_off()
{
  IR_TCCRnA &= ((~(1<<IR_COMn1)) & (~(1<<IR_COMn0)) );
  ir_led_state = 0;
}

void ir_toggle()
{
  if (ir_led_state)
    ir_off();
  else
    ir_on();
}

void ir_start(uint16_t *code)
{
  ir_code = code;
  IR_PORT &= ~IR_BV; // Turn output off
  IR_DDR |= IR_BV; // Set it as output
  IR_TCCRnA = 0x00; // Reset the pwm
  IR_TCCRnB = 0x00;
  uint16_t top = ( (F_CPU/1000000.0) * code[PRONTO_FREQ_CODE] * 0.241246 ) - 1;
  IR_ICRn = top;
  IR_OCRn = top >> 1;
  IR_TCCRnA = (1<<WGM11);
  IR_TCCRnB = (1<<WGM13) | (1<<WGM12);
  IR_TCNTn = 0x0000;
  IR_TIFRn = 0x00;
  IR_TIMSKn = 1 << IR_TOIEn;
  ir_seq_index = PRONTO_CODE_START;
  ir_cycle_count = 0;
  ir_on();
  IR_TCCRnB |= (1<<CS10);
}

#define TOTAL_CYCLES 80000 

ISR(TIMER1_OVF_vect) {
  uint16_t sequenceIndexEnd;
  uint16_t repeatSequenceIndexStart;
  ir_total_cycle_count++;
  ir_cycle_count++;

  if (ir_cycle_count== ir_code[ir_seq_index]) {
    ir_toggle();
    ir_cycle_count = 0;
    ir_seq_index++;
    sequenceIndexEnd = PRONTO_CODE_START +
      (ir_code[PRONTO_SEQUENCE1_LENGTH]<<1) +
      (ir_code[PRONTO_SEQUENCE2_LENGTH]<<1);

    repeatSequenceIndexStart = PRONTO_CODE_START +
      (ir_code[PRONTO_SEQUENCE1_LENGTH]<<1);

    if (ir_seq_index >= sequenceIndexEnd ) {
      ir_seq_index = repeatSequenceIndexStart;

      if(ir_total_cycle_count>TOTAL_CYCLES) {
        ir_off();
        TCCR1B &= ~(1<<CS10);
      }
    }
  }
}

void ir_stop()
{
  IR_TCCRnA = 0x00; // Reset the pwm
  IR_TCCRnB = 0x00;
}

void handleProntoCommand2(const char* received)
{
  // reuse the received buffer... (does arduino require alignment?)
  uint16_t* code = (uint16_t*) received;

  const char* p = received;
  uint16_t j = 0;
  while ( (p = strchr(p, ' ')) != NULL )
    code[j++] = strtol(p, &p, 16);

  digitalWrite(orangeLedPin, HIGH);
  ir_start(code);
  delay(100);
  ir_stop();
  digitalWrite(orangeLedPin, LOW);  
}

#endif //!USE_IRREMOTE_PRONTO
