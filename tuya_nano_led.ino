/*
 * @FileName: start.ino
 * @Author: Tuya
 * @Email: 
 * @LastEditors: Tuya
 * @Date: 2021-04-10 11:24:27
 * @LastEditTime: 2021-04-28 19:48:31
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Description: This demo is based on the Arduino UNO, and the LEDs on the UNO board are controlled by the Tuya Smart App. 
 *               Enter network connection mode when Pin7 to GND.
 * @Github:https://github.com/tuya/tuya-wifi-mcu-sdk-arduino-library
 */

#include <TuyaWifi.h>
#include <SoftwareSerial.h>
#include <FastLED.h>

TuyaWifi my_device;

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define LED_PIN    3
//#define CLK_PIN   4
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    144
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120

/* Current LED status */
unsigned char led_state = 0;//板载灯状态
bool autoChangePatterns = false;//自动切换样式标志位
/* Connect network button pin */
int key_pin = 7;

/* Data point define */
#define DPID_SWITCH 20
#define DPID_MODE 21
#define DPID_BRIGHT 22

//函数声明
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length);
void dp_update_all(void);
void rainbow();
void rainbowWithGlitter();
void confetti();
void sinelon();
void bpm();
void juggle();
void nextPattern();


/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 *                                     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] =
{
  {DPID_SWITCH, DP_TYPE_BOOL},
  {DPID_MODE, DP_TYPE_ENUM},
  {DPID_BRIGHT, DP_TYPE_VALUE},
};

unsigned char pid[] = {"huzf7duveiova6sa"};
unsigned char mcu_ver[] = {"1.0.0"};

uint8_t pattern;
uint8_t brightness = 50;

/* last time */
unsigned long last_time = 0;

void setup() 
{
  // Serial.begin(9600);
  Serial.begin(9600);

  //Initialize led port, turn off led.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //Initialize networking keys.
  pinMode(key_pin, INPUT_PULLUP);

   // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,LED_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(brightness);
  
  //Enter the PID and MCU software version
  my_device.init(pid, mcu_ver);
  //incoming all DPs and their types array, DP numbers
  my_device.set_dp_cmd_total(dp_array, 3);
  //register DP download processing callback function
  my_device.dp_process_func_register(dp_process);
  //register upload all DP callback function
  my_device.dp_update_all_func_register(dp_update_all);

  last_time = millis();
    delay(3000); // 3 second delay for recovery
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 5; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void loop() 
{
  my_device.uart_service();

  //Enter the connection network mode when Pin7 is pressed.
  if (digitalRead(key_pin) == LOW) {
    delay(80);
    if (digitalRead(key_pin) == LOW) {
      my_device.mcu_set_wifi_mode(SMART_CONFIG);
    }
  }
  /* LED blinks when network is being connected */
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) {
    if (millis()- last_time >= 500) {
      last_time = millis();

      if (led_state == LOW) {
        led_state = HIGH;
      } else {
        led_state = LOW;
      }
      digitalWrite(LED_BUILTIN, led_state);
    }
  }

  delay(10);

      //设置运行模式，当开关打开为灯带自动模式，开关关闭为手动调节模式
       if(led_state)
       {FastLED.setBrightness(BRIGHTNESS);
        gPatterns[0]();
       // send the 'leds' array out to the actual LED strip
         FastLED.show();  
       // insert a delay to keep the framerate modest
         FastLED.delay(1000/FRAMES_PER_SECOND); 

       // do some periodic updates
        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
        EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically*/
       }
       else
       {
       FastLED.setBrightness(brightness);//根据下发的值运行闪烁模式
       gPatterns[gCurrentPatternNumber]();//根据下发的值设置灯带的亮度
      // send the 'leds' array out to the actual LED strip
       FastLED.show();  
      // insert a delay to keep the framerate modest
      FastLED.delay(1000/FRAMES_PER_SECOND); 

     // do some periodic updates
     EVERY_N_MILLISECONDS( 10 ) { gHue++; } // slowly cycle the "base color" through the rainbow
     //EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically*/

      Serial.println("当前闪烁模式为：");
      Serial.print(gCurrentPatternNumber);
      Serial.println("当前亮度值为：");
      Serial.print(brightness);    

       delay(100);

     }
}

/**
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  switch(dpid) {
    case DPID_SWITCH:
      led_state = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      if (led_state) {
        //Turn on
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        //Turn off
        digitalWrite(LED_BUILTIN, LOW);
      }
      //Status changes should be reported.
      my_device.mcu_dp_update(dpid, value, length);//上报板载灯状态
    break;
    
    case DPID_MODE:
      gCurrentPatternNumber = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */

      // Call the current pattern function once, updating the 'leds' array
       gPatterns[gCurrentPatternNumber]();
     // send the 'leds' array out to the actual LED strip
       FastLED.show();  
     // insert a delay to keep the framerate modest
      FastLED.delay(1000/FRAMES_PER_SECOND); 
     // do some periodic updates
     EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
     //EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically*/
      my_device.mcu_dp_update(dpid, gCurrentPatternNumber, 1);//上报当前闪烁模式值
      break;

    case DPID_BRIGHT:
      brightness = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      FastLED.setBrightness(brightness);
      //Status changes should be reported.
      my_device.mcu_dp_update(dpid, brightness, 1);//上报当前亮度值
    break;

    default:break;
  }
  return SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
  my_device.mcu_dp_update(DPID_SWITCH, led_state, 1);
  my_device.mcu_dp_update(DPID_MODE, pattern, 1);
  my_device.mcu_dp_update(DPID_BRIGHT, brightness, 1);
}



/*     灯带显示模式      */
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}


void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle()
{
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}
