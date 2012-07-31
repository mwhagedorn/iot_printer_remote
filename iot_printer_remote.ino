// Based off the Adafruit Gutenbird demo
// <https://github.com/adafruit/Adafruit-Tweet-Receipt>.
// 
// MIT License.

#include <SPI.h>
#include <Ethernet.h>
#include <Adafruit_Thermal.h>
#include <SoftwareSerial.h>

// Global stuff --------------------------------------------------------------

const int
  led_pin         = 3,           // To status LED (hardware PWM pin)
  // Pin 4 is skipped -- this is the Card Select line for Arduino Ethernet!
  printer_RX_Pin  = 5,           // Printer connection: green wire
  printer_TX_Pin  = 6,           // Printer connection: yellow wire
  printer_Ground  = 7,           // Printer connection: black wire
  maxTweets       = 5;           // Limit tweets printed; avoid runaway output
const unsigned long              // Time limits, expressed in milliseconds:
  pollingInterval = 60L * 1000L, // Time between polls
  connectTimeout  = 15L * 1000L, // Max time to retry server link
  responseTimeout = 30L * 1000L; // Max time to wait for data from server
Adafruit_Thermal
  printer(printer_RX_Pin, printer_TX_Pin);
byte
  sleepPos = 0, // Current "sleep throb" table position
  resultsDepth, // Used in JSON parsing
  // Ethernet MAC address is found on sticker on Ethernet shield or board:
  mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0xA9, 0x3F };
IPAddress ip(10,0,1,201); // Fallback address -- code will try DHCP first
EthernetClient client;
char
  *serverName  = "iot.appsdynamic.com",
  *url         = "/printer/abc123",
  style,
  justification,
  font_size,
  value[255];    // Temp space for line parsing
  
boolean
  // skip printing when first turned on
  skipPrint = true;

long
  // Time since last print
  timeSincePrint = millis();
  
PROGMEM byte
  sleepTab[] = { // "Sleep throb" brightness table (reverse for second half)
      0,   0,   0,   0,   0,   0,   0,   0,   0,   1,
      1,   1,   2,   3,   4,   5,   6,   8,  10,  13,
     15,  19,  22,  26,  31,  36,  41,  47,  54,  61,
     68,  76,  84,  92, 101, 110, 120, 129, 139, 148,
    158, 167, 177, 186, 194, 203, 211, 218, 225, 232,
    237, 242, 246, 250, 252, 254, 255 };

// Function prototypes -------------------------------------------------------

boolean
  jsonParse(int, byte),
  readString(char *, int),
  parse();
int
  unidecode(byte),
  timedRead(void);

// ---------------------------------------------------------------------------

void setup() {

  // Set up LED "sleep throb" ASAP, using Timer1 interrupt:
  TCCR1A  = _BV(WGM11); // Mode 14 (fast PWM), 64:1 prescale, OC1A off
  TCCR1B  = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  ICR1    = 8333;       // ~30 Hz between sleep throb updates
  TIMSK1 |= _BV(TOIE1); // Enable Timer1 interrupt
  sei();                // Enable global interrupts

  Serial.begin(57600);
  pinMode(printer_Ground, OUTPUT);
  digitalWrite(printer_Ground, LOW);  // Just a reference ground, not power
  printer.begin();
  printer.sleep();

  // Initialize Ethernet connection.  Request dynamic
  // IP address, fall back on fixed IP if that fails:
  Serial.print("Initializing Ethernet...");
  if(Ethernet.begin(mac)) {
    Serial.println("OK");
  } else {
    Serial.print("\r\nno DHCP response, using static IP address.");
    Ethernet.begin(mac, ip);
  }
  memset(value    , 0, sizeof(value));
}

// ---------------------------------------------------------------------------

void loop() {
  unsigned long startTime, t;

  startTime = millis();

  // Disable Timer1 interrupt during network access, else there's trouble.
  // Just show LED at steady 100% while working.  :T
  TIMSK1 &= ~_BV(TOIE1);
  analogWrite(led_pin, 255);

  // Attempt server connection, with timeout...
  Serial.print("Connecting to server...");
  while((client.connect(serverName, 80) == false) &&
    ((millis() - startTime) < connectTimeout));

  if(client.connected()) { // Success!
    Serial.print("OK\r\nIssuing HTTP request...");
    // URL-encode queryString to client stream:
    client.print("GET ");
    client.print(url);
    client.print(" HTTP/1.1\r\nHost: ");
    client.println(serverName);
    client.println("Connection: close\r\n");

    Serial.print("OK\r\nAwaiting results (if any)...");
    t = millis();
    while((!client.available()) && ((millis() - t) < responseTimeout));
    if(client.available()) { // Response received?
      if(client.find("\r\n\r\n")) { // Skip HTTP response header
        Serial.println("OK\r\nProcessing results...");
        printer.wake();
        if (parse()) {
          printer.feed(3);
        };
        printer.sleep();
      } else Serial.println("response not recognized.");
    } else   Serial.println("connection timed out.");
    client.stop();
  } else { // Couldn't contact server
    Serial.println("failed");
  }

  // Sometimes network access & printing occurrs so quickly, the steady-on
  // LED wouldn't even be apparent, instead resembling a discontinuity in
  // the otherwise smooth sleep throb.  Keep it on at least 4 seconds.
  t = millis() - startTime;
  if(t < 4000L) delay(4000L - t);

  // Pause between queries, factoring in time already spent on network
  // access, parsing, printing and LED pause above.
  t = millis() - startTime;
  if(t < pollingInterval) {
    Serial.print("Pausing...");
    sleepPos = sizeof(sleepTab); // Resume following brightest position
    TIMSK1 |= _BV(TOIE1); // Re-enable Timer1 interrupt for sleep throb
    delay(pollingInterval - t);
    Serial.println("done");
  }
}

// ---------------------------------------------------------------------------

boolean parse() {
  int c;
  boolean readLine = false;
  boolean styleFound = false;
  boolean content = false;
  
  for(;;) {
    // Squash any whitespace
    if (!readLine) while(isspace(c = timedRead()));
    if(c < 0)  return content; // Timeout
    if(c == 0) return content;  // EOD
        
    if (readLine) {
      Serial.println("in readline");
      content = true;
      // Read the rest of the line
      readString(value, sizeof(value)-1);
      Serial.println(value);
      Serial.println("");
      // Print it
       printer.println(value);
//      if (style == 'n') {
//        printer.println(value);
//      } else {
//        printer.print(value);
//      }
      // Reset printer values
      switch(style) {
        case 'b':
          printer.boldOff();
          break;
        case 'u':
          printer.underlineOff();
          break;
        case 'i':
          printer.inverseOff();
          break;
      }
      readLine = false;
    } else {
      Serial.println("in else readline");
      styleFound = true;
      style = c;
      justification = timedRead();

      // Squash space
      timedRead();
      
      switch (style) {
        case 'b':
          Serial.println("Setting to bold");
          printer.boldOn();
          break;
        case 'u':
          Serial.println("Setting to underline");
          printer.underlineOn();
          break;
        case 'i':
          Serial.println("Setting to inverse");
          printer.inverseOn();
          break;
        case 'n':
          Serial.println("Normal line");
          break;
        default:
          styleFound = false;
      }
      if (styleFound) {
        Serial.print("Justification: ");
        Serial.write(toupper(justification));
        Serial.println("");
        printer.justify(toupper(justification));
        readLine = true;
      }
    }
  } 
}

// ---------------------------------------------------------------------------

// Read string from client stream into destination buffer, up to a maximum
// requested length.  Buffer should be at least 1 byte larger than this to
// accommodate NUL terminator.  Opening quote is assumed already read,
// closing quote will be discarded, and stream will be positioned
// immediately following the closing quote (regardless whether max length
// is reached -- excess chars are discarded).  Returns true on success
// (including zero-length string), false on timeout/read error.
boolean readString(char *dest, int maxLen) {
  int c, len = 0;
  while((c = timedRead()) != '\n' && (c != '\r')) { // Read until newline 
    if(c < 0) return false; // Timeout
    if(len < maxLen) dest[len++] = c;
  }

  dest[len] = 0;
  return true; // Success (even if empty string)
}

// ---------------------------------------------------------------------------

// Read a given number of hexadecimal characters from client stream,
// representing a Unicode symbol.  Return -1 on error, else return nearest
// equivalent glyph in printer's charset.  (See notes below -- for now,
// always returns '-' or -1.)
int unidecode(byte len) {
  int c, v, result = 0;
  while(len--) {
    if((c = timedRead()) < 0) return -1; // Stream timeout
    if     ((c >= '0') && (c <= '9')) v =      c - '0';
    else if((c >= 'A') && (c <= 'F')) v = 10 + c - 'A';
    else if((c >= 'a') && (c <= 'f')) v = 10 + c - 'a';
    else return '-'; // garbage
    result = (result << 4) | v;
  }

  // To do: some Unicode symbols may have equivalents in the printer's
  // native character set.  Remap any such result values to corresponding
  // printer codes.  Until then, all Unicode symbols are returned as '-'.
  // (This function still serves an interim purpose in skipping a given
  // number of hex chars while watching for timeouts or malformed input.)

  return '-';
}

// ---------------------------------------------------------------------------

// Read from client stream with a 5 second timeout.  Although an
// essentially identical method already exists in the Stream() class,
// it's declared private there...so this is a local copy.
int timedRead(void) {
  int           c;
  unsigned long start = millis();

  while((!client.available()) && ((millis() - start) < 5000L));
  
  return client.read();
}

// ---------------------------------------------------------------------------

// Timer1 interrupt handler for sleep throb
ISR(TIMER1_OVF_vect, ISR_NOBLOCK) {
  // Sine table contains only first half...reflect for second half...
  analogWrite(led_pin, pgm_read_byte(&sleepTab[
    (sleepPos >= sizeof(sleepTab)) ?
    ((sizeof(sleepTab) - 1) * 2 - sleepPos) : sleepPos]));
  if(++sleepPos >= ((sizeof(sleepTab) - 1) * 2)) sleepPos = 0; // Roll over
  TIFR1 |= TOV1; // Clear Timer1 interrupt flag
}
