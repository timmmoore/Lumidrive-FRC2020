#include <SoftWire.h>

// Modified Adafruit example code for fastled lib
#include <FastLED.h>

#define Serial SERIAL_PORT_USBVIRTUAL // map Serial to the USB serial device
//#define WAITUSB                       // uncomment if you want to wait for serial monitor before starting

#define NUM_LEDS    60

#define DATA_PIN    10
#define CLK_PIN     12

//#define LOUD_SENSOR                   // uncomment to enable loud sensor
#define LOUD_DEBUG

#define MIN_BRIGHT 4
#define MAX_BRIGHT 80
#define BRIGHTNESS 40                 // global brightness 0-255, increasing this impacts power requirements of LED strip
#define MAX_INPUT_BRIGHTNESS 128      // max brightness allowed to be set, to limit current draw
#define LED_TYPE    DOTSTAR           // type of strip
#define COLOR_ORDER BGR               // Order of leds in strip
#define MAX_POWER_MILLIAMPS 1500

#define UPDATERATE  50                // uncomment if you want to slow the led update rate down, ~3.5us per led is full speed

#define LED 13                        // status LED
#define BUTTON 6                      // button
#define CONTROL 8                     // input pin

#define NEUTRAL   1                   // SCAN same foreground/background colors
#define SCAN_RED  2                   // SCAN red foreground/blue background colors
#define SCAN_BLUE 3                   // SCAN blue foreground/red background colors
#define SCAN_WHITE 4                  // SCAN white foreground/green background colors
#define RAINBOW 5
#define START_TYPE NEUTRAL
#define END_TYPE RAINBOW
#define INIT_TYPE RAINBOW             // initial scan type

CRGB leds[NUM_LEDS];

uint8_t type = 0;                     // initialize to invalid scan type
uint8_t fr = 255, fg = 0, fb = 0;     // foreground color
uint8_t br = 0, bg = 0, bb = 255;     // background color
int16_t brightness = BRIGHTNESS;
#define BRIGHTCOUNT 2
uint8_t maxbrightcount = BRIGHTCOUNT;
uint8_t maxbright = MAX_BRIGHT;
uint8_t minbright = MIN_BRIGHT;

#define COMMAND_LED_OFF       0x00
#define COMMAND_LED_ON        0x01
#define COMMAND_GET_VALUE     0x05
#define COMMAND_NOTHING_NEW   0x99

#if defined(LOUD_SENSOR)
SoftWire sw(A3, 9);                 // SDA: A3, SCL: 9
#endif

uint8_t qwiicAddress = 0x38;        // Default loudness sensor address
uint8_t loudness = false;

void fill()
{
  for(uint16_t i = 0; i < NUM_LEDS; i++)
    leds[i].setRGB(br, bg, bb);
}

// testForLoudSensor() checks for an ACK from an Sensor. If no ACK
void testForLoudSensor()
{
#if defined(LOUD_SENSOR)
#if defined(LOUD_DEBUG)
  Serial.println("Loudness sensor beginTransmission");
#endif
  sw.beginTransmission(qwiicAddress);
#if defined(LOUD_DEBUG)
  Serial.println("Loudness sensor endTransmission");
#endif
  if (sw.endTransmission() != 0)    //check here for an ACK from the slave, if no ACK don't allow change?
  {
#if defined(LOUD_DEBUG)
    Serial.println("Loudness sensor not found");
#endif
    loudness = false;
  }
  else
  {
#if defined(LOUD_DEBUG)
    Serial.println("Loudness sensor found");
#endif
    loudness = true;
  }
#endif
}

// LED is off, and a -1 if an error occurred.
uint16_t get_value()
{
  uint16_t ADC_VALUE = 0;
#if defined(LOUD_SENSOR)
  sw.beginTransmission(qwiicAddress);
  sw.write(COMMAND_GET_VALUE);      // command for status
  sw.endTransmission();             // stop transmitting //this looks like it was essential.

  sw.requestFrom(qwiicAddress, (uint8_t)2);  // request 1 bytes from slave device qwiicAddress

  while (sw.available())            // slave may send less than requested
  {
    uint8_t ADC_VALUE_L = sw.read();
    uint8_t ADC_VALUE_H = sw.read();
    ADC_VALUE = ADC_VALUE_H;
    ADC_VALUE <<= 8;
    ADC_VALUE |= ADC_VALUE_L;
  }
  uint16_t x = sw.read();
#endif
  return ADC_VALUE;
}

void setup() {
  Serial.begin(115200);
#ifdef WAITUSB
  while(!Serial);                     // if debugging to wait for USB serial monitor before starting
  Serial.println("Lumidrive");
#endif

  pinMode(LED, OUTPUT);               // led pin is output
  pinMode(BUTTON, INPUT_PULLUP);      // button pin is input, enable pullup
  pinMode(CONTROL, INPUT_PULLUP);     // input pin, enable pullup

#if defined(LOUD_SENSOR)
#if defined(LOUD_DEBUG)
  Serial.println("Loudness sensor begin");
#endif
  sw.begin();
#if defined(LOUD_DEBUG)
  Serial.println("Loudness sensor begin done");
#endif

#endif
  testForLoudSensor();

  FastLED.addLeds<LED_TYPE, DATA_PIN,CLK_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInVoltsAndMilliamps( 5, MAX_POWER_MILLIAMPS);
  FastLED.setBrightness(brightness);  // global led brightness, dont remove without understanding power requirements
  change_type(INIT_TYPE);             // initialize for initial type
  FastLED.show();                     // set all LEDs to background color
}

char buffer [16];                     // input buffer
int red, green, blue;                 // input colors
char com;                             // input command
char *ptr;

// flip background/foreground colors
void invert()
{
  uint8_t t;
  t = fr; fr = br; br = t;
  t = fg; fg = bg; bg = t;
  t = fb; fb = bb; bb = t;
  fill();                             // fill all leds with background
}

// init the type of scan we want
void change_type(uint8_t new_type)
{
  uint8_t old_type = type;

  if((type = new_type) != old_type)
  {
    switch(type)
    {
    case SCAN_BLUE:
    case SCAN_RED:
    case SCAN_WHITE:
      init_scan();
      break;
    case RAINBOW:
      init_rainbow();
      break;
    default:
      type = NEUTRAL;                 // in case its not NEUTRAL
      init_neutral();                 // init neutral scan
      break;
    }
  }
}
// parse input line
void ParseLine()
{
  com = buffer[0];                    // get command char
  ptr = strtok(&buffer[1],",");       // 1st parameter
  if(ptr)
    red = atoi(ptr);
  else
    red = -1;
  ptr = strtok(NULL,",");             // 2nd parameter
  if(ptr)
    green = atoi(ptr);
  else
    green = -1;
  ptr = strtok(NULL,NULL);            // 3rd parameter
  if(ptr)
    blue = atoi(ptr);
  else
    blue = -1;
 
  if(com == 'f')                      // set foreground color
  {
    if(red != -1) fr = red;
    if(green != -1) fg = green;
    if(blue != -1) fb = blue;
  }
  else if(com == 'b')                 // set background color
  {
    if(red != -1) br = red;
    if(green != -1) bg = green;
    if(blue != -1) bb = blue;
    fill();                           // fill all leds with background
  }
  else if(com == 'r')                 // speed of changing broghtness
  {
    if(red != -1)
      maxbrightcount = red;           // speed of brightness change
  }
  else if(com == 'i')                 // set minimum brightness
  {
    if(red != -1)
      minbright = red;                // min brightness
  }
  else if(com == 'a')                 // set maximum brightness
  {
    if(red != -1)
      maxbright = red;                // max brightness
    if(maxbright > MAX_INPUT_BRIGHTNESS) maxbright = MAX_INPUT_BRIGHTNESS;
  }
  else if(com == 'v')                 // flip foreground/background colors
  {
    invert();
  }
  else if(com == 't')                 // set scan type
  {
    if(red != -1)
      change_type(red);               // 1 is pacifica, 2 is red scan, 3 is blue scan, 4 is white scan, 5 is rainbow
    else
      next_type();
  }
}

uint8_t ledo = HIGH;                  // status led state
int16_t head  = 0, tail = -10;        // Index of head and tail of foreground color, note: the -ve tail
boolean headdirection = true;
uint8_t cnt = 0;                      // position in input buffer
boolean ready = false;                // have input line to process
uint8_t inputcheck = 0;               // limit how often we check for input
#define INPUTCOUNT  50
uint8_t brightcount = 0;              // how often to change brightness
boolean brightdirection = false;      // brightness going up or down
uint8_t buttonread = 0;               // 0 - currently high, 1 changing to low, 2 currently low, 3 chaning to high
uint8_t controloutput = 0;            // 0 - currently high, 1 changing to low, 2 currently low, 3 chaning to high

boolean debounce(boolean value, uint8_t *state)
{
  boolean change = false;
  switch(*state)                      // check state and value and debounce
  {
    case 0:
      if(value == false)              // first low
        *state = 1;
      break;
    case 1:
      if(value == false)              // still low
      {
        *state = 2;
        change = true;
      }
      else
        *state = 0;                   // gone back high
      break;
    case 2:
      if(value == true)               // first high
        *state = 3;
      break;
    case 3:
      if(value == true)               // still high
      {
        *state = 0;
        change = true;
       }
      else
        *state = 2;                   // gone back low
      break;
    default:
      *state = 0;
  }
  return change;
}

void next_type()
{
  uint8_t next_type = type + 1;
  if(next_type > END_TYPE) next_type = START_TYPE;
  change_type(next_type);
}

void loop()
{
  if(debounce(digitalRead(CONTROL), &controloutput))
    next_type();                      // cycle scan types
  else if(debounce(digitalRead(BUTTON), &buttonread) && (buttonread == 2))
    next_type();                      // cycle scan types only during down press
  if(++inputcheck == INPUTCOUNT)
  {
    while (Serial.available())        // read if input available
    {
      char c = Serial.read();
      buffer[cnt++] = c;
      if ((c == '\n') || (cnt == sizeof(buffer)-1))
      {
        buffer[cnt] = '\0';           // either buffer full or end of line
        cnt = 0;
        ready = true;
      }
    }
    inputcheck = 0;
  }
  if (ready)                          // if have line parse it
  {
    ParseLine();
    ready = false;
  }
  digitalWrite(LED, ledo);            // set status led

  uint8_t status = false;
  switch(type)                        // do the right type of led scan
  {
  case NEUTRAL:
    status = neutral();
    break;
  case RAINBOW:
    status = do_rainbox();
    break;
  default:
    status = do_scan();
    break;
  }
  if(status)
    ledo = (ledo==HIGH)?LOW:HIGH;     // flip led status
}

void init_neutral()
{
  fr = 0, fg = 0, fb = 0;             // foreground color - black
  br = 0, bg = 0, bb = 0;             // background color - black
  fill();
  brightness = MAX_BRIGHT;
}

uint8_t neutral()
{
#if defined(LOUD_SENSOR)
  if(loudness == true)
  {
    uint32_t b;
    b = get_value();
    b *= MAX_BRIGHT;
    b >>= 10;
    brightness = b;
  }
#endif
  FastLED.setBrightness(brightness);  // global led brightness, dont remove without understanding power requirements
  uint8_t status = pacifica_loop();
  FastLED.show();
#ifdef UPDATERATE
  FastLED.delay(1000/UPDATERATE);     // if you want/need to slow the update down, handles led dithering etc. while waiting
#endif
  return status;
}

void init_rainbow()
{
  brightness = MAX_BRIGHT;
}

uint8_t do_rainbox()
{
#if defined(LOUD_SENSOR)
  if(loudness == true)
  {
    uint32_t b;
    b = get_value();
    b *= MAX_BRIGHT;
    b >>= 10;
    brightness = b;
  }
#endif
  FastLED.setBrightness(brightness);  // global led brightness, dont remove without understanding power requirements
  uint8_t status = false;
  static uint32_t total = 0;
  static uint32_t sLastms = 0;
  uint32_t ms = GET_MILLIS();
  total += (ms - sLastms);
  sLastms = ms;
  if(total > 1000)
  {
    total = 0;
    status = true;
  }
  static uint8_t startIndex = 0;
  startIndex = startIndex + 1; /* motion speed */
    
  FillLEDsFromPaletteColors(startIndex);
    
  FastLED.show();
  FastLED.delay(1000/UPDATERATE);
  return status;
}

void FillLEDsFromPaletteColors(uint8_t colorIndex)
{
  for(int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette(RainbowColors_p, colorIndex, brightness, LINEARBLEND);
    colorIndex += 3;
  }
}

void init_scan()
{
  if(type == SCAN_RED)                // foreground red
  {
    fr = 255, fg = 0, fb = 0;         // foreground color - red
    br = 0, bg = 0, bb = 255;         // background color - blue
  }
  else if(type == SCAN_BLUE)
  {
    fr = 0, fg = 0, fb = 255;         // foreground color - blue
    br = 255, bg = 0, bb = 0;         // background color - red
  }
  else
  {
    fr = 255, fg = 255, fb = 255;     // foreground color - white
    br = 0, bg = 255, bb = 0;         // background color - green
  }
  fill();
}

uint8_t do_scan()
{
  uint8_t status = false;
  if(headdirection)                   // update led at head/tail of change
  {
    if((head >= 0) && (head < NUM_LEDS))
      leds[head].setRGB(fr, fg, fb);  // foreground color pixel at head
    if((tail >= 0) && (tail < NUM_LEDS))
      leds[tail].setRGB(br, bg, bb);  // background color pixel at tail
  }
  else
  {
    if((head >= 0) && (head < NUM_LEDS))
      leds[head].setRGB(br, bg, bb);  // background color pixel at head
    if((tail >= 0) && (tail < NUM_LEDS))
      leds[tail].setRGB(fr, fg, fb);  // foreground color pixel at tail
  }
  FastLED.show();                     // ~3.5us per led
#ifdef UPDATERATE
  FastLED.delay(1000/UPDATERATE);     // if you want/need to slow the update down, handles led dithering etc. while waiting
#endif
  if(headdirection)                   // update head/tail pointers
  {
    ++head;                           // Increment head index.  Off end of strip?
    if(++tail >= NUM_LEDS)
    {
      headdirection = false;          // tail at end of strip
      status = true;
    }
  }
  else
  {
    --tail;
    if(--head <= 0)                   // head at beginning of strip
    {
      headdirection = true;
      ledo = (ledo==HIGH)?LOW:HIGH;   // flip led status each time through the strip
    }    
  }
  if(++brightcount == maxbrightcount) // update brightness up/down
  {
    if(brightdirection)               // dimmer
    {
      if(--brightness < minbright)
        brightdirection = false;      // change to going up
    }
    else
    {
      if(++brightness > maxbright)
        brightdirection = true;       // change to going down
    }
    FastLED.setBrightness(brightness);// global led brightness, dont remove without understanding power requirements
    brightcount = 0;
  }
  return status;
}

//////////////////////////////////////////////////////////////////////////
//
// In this animation, there are four "layers" of waves of light.  
//
// Each layer moves independently, and each is scaled separately.
//
// All four wave layers are added together on top of each other, and then 
// another filter is applied that adds "whitecaps" of brightness where the 
// waves line up with each other more.  Finally, another pass is taken
// over the led array to 'deepen' (dim) the blues and greens.
//
// The speed and scale and motion each layer varies slowly within independent 
// hand-chosen ranges, which is why the code has a lot of low-speed 'beatsin8' functions
// with a lot of oddly specific numeric ranges.
//
// These three custom blue-green color palettes were inspired by the colors found in
// the waters off the southern coast of California, https://goo.gl/maps/QQgd97jjHesHZVxQ7
//
CRGBPalette16 pacifica_palette_1 = 
    { 0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117, 
      0x000019, 0x00001C, 0x000026, 0x000031, 0x00003B, 0x000046, 0x14554B, 0x28AA50 };
CRGBPalette16 pacifica_palette_2 = 
    { 0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117, 
      0x000019, 0x00001C, 0x000026, 0x000031, 0x00003B, 0x000046, 0x0C5F52, 0x19BE5F };
CRGBPalette16 pacifica_palette_3 = 
    { 0x000208, 0x00030E, 0x000514, 0x00061A, 0x000820, 0x000927, 0x000B2D, 0x000C33, 
      0x000E39, 0x001040, 0x001450, 0x001860, 0x001C70, 0x002080, 0x1040BF, 0x2060FF };


uint8_t pacifica_loop()
{
  uint8_t status = false;
  // Increment the four "color index start" counters, one for each wave layer.
  // Each is incremented at a different speed, and the speeds vary over time.
  static uint16_t sCIStart1, sCIStart2, sCIStart3, sCIStart4;
  static uint32_t sLastms = 0;
  static uint32_t total = 0;
  uint32_t ms = GET_MILLIS();
  uint32_t deltams = ms - sLastms;
  sLastms = ms;
  uint16_t speedfactor1 = beatsin16(3, 179, 269);
  uint16_t speedfactor2 = beatsin16(4, 179, 269);
  uint32_t deltams1 = (deltams * speedfactor1) / 256;
  uint32_t deltams2 = (deltams * speedfactor2) / 256;
  uint32_t deltams21 = (deltams1 + deltams2) / 2;
  sCIStart1 += (deltams1 * beatsin88(1011,10,13));
  sCIStart2 -= (deltams21 * beatsin88(777,8,11));
  sCIStart3 -= (deltams1 * beatsin88(501,5,7));
  sCIStart4 -= (deltams2 * beatsin88(257,4,6));

  // Clear out the LED array to a dim background blue-green
  fill_solid( leds, NUM_LEDS, CRGB( 2, 6, 10));

  // Render each of four layers, with different scales and speeds, that vary over time
  pacifica_one_layer( pacifica_palette_1, sCIStart1, beatsin16( 3, 11 * 256, 14 * 256), beatsin8( 10, 70, 130), 0-beat16( 301) );
  pacifica_one_layer( pacifica_palette_2, sCIStart2, beatsin16( 4,  6 * 256,  9 * 256), beatsin8( 17, 40,  80), beat16( 401) );
  pacifica_one_layer( pacifica_palette_3, sCIStart3, 6 * 256, beatsin8( 9, 10,38), 0-beat16(503));
  pacifica_one_layer( pacifica_palette_3, sCIStart4, 5 * 256, beatsin8( 8, 10,28), beat16(601));

  // Add brighter 'whitecaps' where the waves lines up more
  pacifica_add_whitecaps();

  // Deepen the blues and greens a bit
  pacifica_deepen_colors();

  total += deltams;
  if(total > 1000)
  {
    total = 0;
    status = true;
  }
  return status;
}

// Add one layer of waves into the led array
void pacifica_one_layer( CRGBPalette16& p, uint16_t cistart, uint16_t wavescale, uint8_t bri, uint16_t ioff)
{
  uint16_t ci = cistart;
  uint16_t waveangle = ioff;
  uint16_t wavescale_half = (wavescale / 2) + 20;
  for( uint16_t i = 0; i < NUM_LEDS; i++) {
    waveangle += 250;
    uint16_t s16 = sin16( waveangle ) + 32768;
    uint16_t cs = scale16( s16 , wavescale_half ) + wavescale_half;
    ci += cs;
    uint16_t sindex16 = sin16( ci) + 32768;
    uint8_t sindex8 = scale16( sindex16, 240);
    CRGB c = ColorFromPalette( p, sindex8, bri, LINEARBLEND);
    leds[i] += c;
  }
}

// Add extra 'white' to areas where the four layers of light have lined up brightly
void pacifica_add_whitecaps()
{
  uint8_t basethreshold = beatsin8( 9, 55, 65);
  uint8_t wave = beat8( 7 );
  
  for( uint16_t i = 0; i < NUM_LEDS; i++) {
    uint8_t threshold = scale8( sin8( wave), 20) + basethreshold;
    wave += 7;
    uint8_t l = leds[i].getAverageLight();
    if( l > threshold) {
      uint8_t overage = l - threshold;
      uint8_t overage2 = qadd8( overage, overage);
      leds[i] += CRGB( overage, overage2, qadd8( overage2, overage2));
    }
  }
}

// Deepen the blues and greens
void pacifica_deepen_colors()
{
  for( uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i].blue = scale8( leds[i].blue,  145); 
    leds[i].green= scale8( leds[i].green, 200); 
    leds[i] |= CRGB( 2, 5, 7);
  }
}
