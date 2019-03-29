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
 * 1) SEND: take such ASCII commands via the serial ports and send them as IR signals.
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
 * 2) open serial port with 9600 baud
 * 3) direct an IR remote at the IR detector
 * 4) you will see all detected IR commands, e.g.:
 *    NEC 20F0B54A
 *    JVC LEN:16 20F0
 *    PANASONIC ADDR:5362 9164AD03
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

#include <IRremote.h>
#include <irPronto.cpp>

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
  Serial.begin(9600);
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

//does not work:
//#define CF(s)  (const char*)F(s)
#define CF(s)  s

const char* encoding2string(decode_type_t type)
{
  switch (type) {
    case RC5:          return CF("RC5");
    case RC6:          return CF("RC6");
    case NEC:          return CF("NEC");
    case SONY:         return CF("SONY");
    case PANASONIC:    return CF("PANASONIC");
    case JVC:          return CF("JVC");
    case SAMSUNG:      return CF("SAMSUNG");
    case WHYNTER:      return CF("WHYNTER");
    case AIWA_RC_T501: return CF("AIWA");
    case LG:           return CF("LG");
    case SANYO:        return CF("SANYO");
    case MITSUBISHI:   return CF("MITSUBISHI");
    case DISH:         return CF("DISH");
    case SHARP:        return CF("SHARP");
    case DENON:        return CF("DENON");
    case PRONTO:       return CF("PRONTO");
    case LEGO_PF:      return CF("LEGO");
  }
  return NULL;
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
  // skip white space
  while (*(*received) != 0 && *(*received) <= 32)
  {
    (*received)++;
  }
  int len = strlen(command);
  if (strncmp(*received, command, len) == 0)
  {
    (*received) += len;
    // skip white space
    while (*(*received) != 0 && *(*received) <= 32)
    {
      (*received)++;
    }
    return true;
  }
  return false;
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
  if (results->decode_type == NEC && results->value == REPEAT)
  {
    // Don't record a NEC repeat value as that's useless.
    Serial.println(F("NEC REPEAT"));
    return;
  }

  Serial.print(typeString);
  Serial.print(F(" "));
  if (results->decode_type == PANASONIC)
  {
    Serial.print(F("ADDR:"));
    Serial.print(results->address, HEX);
    Serial.print(F(" "));
  }
  if (results->bits != 32)
  {
    Serial.print(F("LEN:"));
    Serial.print(results->bits, DEC);
    Serial.print(F(" "));
  }
  Serial.println(results->value, HEX);
}

unsigned long parseValue(const char** command, int base)
{
  const char* p = *command;
  unsigned long ret = strtol(*command, &p, base);
  if (*command == p)
  {
    // parse error
    return 0;
  }
  *command = p;
  // skip white space
  while (*(*command) != 0 && *(*command) <= 32)
  {
    (*command)++;
  }
  return ret;
}

void handleCommand(decode_type_t type, const char* command)
{
  static int toggle = 0; // The RC5/6 toggle state

  unsigned int address = 0;
  unsigned long value = 0;
  int bits = 32;
  bool repeat = false;
  if (type != PRONTO)
  {
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
  }

  digitalWrite(orangeLedPin, HIGH);
  // now send
  switch (type)
  {
  case RC5: // fall through
  case RC6:
  {
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
  case PRONTO:
  {
    if (!irsend.sendPronto((char*)command, PRONTO_REPEAT, PRONTO_FALLBACK))
    {
      Serial.println(F("ERROR PRONTO"));
    }
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

void handleReceivedCommand(const char* received)
{
  bool ok = false;

  // parse IR prefixes
  for (int t = 0; t < decode_type_t_count; t++)
  {
    if (startsWithCommand(&received, encoding2string((decode_type_t)t)))
    {
      handleCommand((decode_type_t)t, received);
      ok = true;
      break;
    }
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
        Serial.println(F("INPUT BUFFER OVERFLOW"));
      }
      readCharCount++;
    }
  }
}
