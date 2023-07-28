/****************************************************************************************************************************
  P1AMrewinder.ino
  For SAMD boards
  Mike Timmons ( derived from Written by Khoi Hoang as...)
  
  Built by Khoi Hoang https://github.com/khoih-prog/SAMD_TimerInterrupt
  Licensed under MIT license
  
  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one SAMD timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.
*****************************************************************************************************************************/
/*
   Notes:
   Special design is necessary to share data between interrupt code and the rest of your program.
   Variables usually need to be "volatile" types. Volatile tells the compiler to avoid optimizations that assume
   variable can not spontaneously change. Because your function may change variables while your program is using them,
   the compiler needs this hint. But volatile alone is often not enough.
   When accessing shared variables, usually interrupts must be disabled. Even with volatile,
   if the interrupt changes a multi-byte variable between a sequence of instructions, it can be read incorrectly.
   If your data is multiple variables, such as an array and a count, usually interrupts need to be disabled
   or the entire sequence of your code which accesses the data.
*/

#if !( defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRWIFI1010) \
      || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310) \
      || defined(ARDUINO_SAMD_MKRGSM1400) || defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) \
      || defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(__SAMD51__) || defined(__SAMD51J20A__) \
      || defined(__SAMD51J19A__) || defined(__SAMD51G19A__) || defined(__SAMD51P19A__)  \
      || defined(__SAMD21E15A__) || defined(__SAMD21E16A__) || defined(__SAMD21E17A__) || defined(__SAMD21E18A__) \
      || defined(__SAMD21G15A__) || defined(__SAMD21G16A__) || defined(__SAMD21G17A__) || defined(__SAMD21G18A__) \
      || defined(__SAMD21J15A__) || defined(__SAMD21J16A__) || defined(__SAMD21J17A__) || defined(__SAMD21J18A__) )
  #error This code is designed to run on SAMD21/SAMD51 platform! Please check your Tools->Board setting.
#endif

/////////////////////////////////////////////////////////////////

// These define's must be placed at the beginning before #include "SAMDTimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// Don't define TIMER_INTERRUPT_DEBUG > 2. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

// Select only one to be true for SAMD21. Must must be placed at the beginning before #include "SAMDTimerInterrupt.h"
#define USING_TIMER_TC3         true      // Only TC3 can be used for SAMD51
#define USING_TIMER_TC4         false     // Not to use with Servo library
#define USING_TIMER_TC5         false
#define USING_TIMER_TCC         false
#define USING_TIMER_TCC1        false
#define USING_TIMER_TCC2        false     // Don't use this, can crash on some boards

// Uncomment To test if conflict with Servo library
//#include "Servo.h"

/////////////////////////////////////////////////////////////////

#include "SAMDTimerInterrupt.h"
#include <P1AM.h>
#include <P1_HSC.h>
#include <LibPrintf.h>

#ifndef LED_BUILTIN
  #define LED_BUILTIN       13
#endif

#define P104AD_MODULE_NUM 1
#define COUNT_RANGE 65535

#define LVDT_VIN_CHANNEL 1
#define EDGE_VIN_CHANNEL 2
#define CMD_MAX  5.0
#define CMD_MIDRANGE 5.0

#define LVDT_VOLTS(x)  (10.0*x/COUNT_RANGE)
#define EDGE_VOLTS(x)  (20.0*x/COUNT_RANGE)
#define CMD_COUNTS(x)   (x*4095/10.0)
#define DLVDT(x1,x2)     ((x2-x1)/(0.01))
unsigned int SWPin = 7;

// TC3, TC4, TC5 max permissible TIMER_INTERVAL_MS is 1398.101 ms, larger will overflow, therefore not permitted
// Use TCC, TCC1, TCC2 for longer TIMER_INTERVAL_MS
#define SAMPLE_RATE              100
#define TIMER_INTERVAL_MS        (1000/SAMPLE_RATE)

volatile uint32_t preMillisTimer = 0;

float freq=5.0;
float amp=2000;
float y = 0;
int   ticks=0;

float lvdtVin[2];
float ddtLvdt = 0;
float edgeVin = 0.0;

float Kpe = .0254; 
float perr=0.0;
float serr=0.0;
float u=0.0;
float ui=0.0;
float cmd=0.0;

// gain scaling for web speed
float Kpmax = 7;
float wcmax = 2;
float wcdes = wcmax;  // initialize desired wc and don't set it until speed is non-zero
float tdelay = 0;
float bz = 0.0842;   // zero location
float Kp=0;
float Ki=0;

/* generated using tool at https://facts-engineering.github.io/modules/P1-04AD/P1-04AD.html
 *  Channel 1=0-10V
 *  Channel 2=+-10V
 *  Channel 3=+-10V
 *  Channel 4=+-10V
 */
const char P1_04AD_CONFIG[] = { 0x40, 0x03, 0x00, 0x00, 0x20, 0x01, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x23, 0x00 };

///////////////////////////////////////////////

#if (TIMER_INTERRUPT_USING_SAMD21)

  #if USING_TIMER_TC3
    #define SELECTED_TIMER      TIMER_TC3
  #elif USING_TIMER_TC4
    #define SELECTED_TIMER      TIMER_TC4
  #elif USING_TIMER_TC5
    #define SELECTED_TIMER      TIMER_TC5
  #elif USING_TIMER_TCC
    #define SELECTED_TIMER      TIMER_TCC
  #elif USING_TIMER_TCC1
    #define SELECTED_TIMER      TIMER_TCC1
  #elif USING_TIMER_TCC2
    #define SELECTED_TIMER      TIMER_TCC
  #else
    #error You have to select 1 Timer  
  #endif

#else

  #if !(USING_TIMER_TC3)
    #error You must select TC3 for SAMD51
  #endif
  
  #define SELECTED_TIMER      TIMER_TC3

#endif  

// Init selected SAMD timer
SAMDTimer ITimer(SELECTED_TIMER);

///////////float y = 0;
void TimerHandler()
{
  static bool toggle = false;
  // Read
  lvdtVin[1] = LVDT_VOLTS(P1.readAnalog(1,LVDT_VIN_CHANNEL));
  edgeVin    = EDGE_VOLTS(P1.readAnalog(1,EDGE_VIN_CHANNEL));

  // LVDT rate
  ddtLvdt = DLVDT(lvdtVin[0],lvdtVin[1]);
  lvdtVin[0] = lvdtVin[1];  // propagate LVDT state

  perr = -edgeVin*Kpe;   // map edge guide voltage to LVDT equivalent


	// P term
	u = Kp*perr;

	// I term
	ui = ui + (Ki*perr)/SAMPLE_RATE;

  // clamp integrator
  ui = max(ui, -CMD_MAX);
  ui = min(ui, CMD_MAX);

  // speed error
	serr = (u + ui) - ddtLvdt;

	// unity gain on inner loop...consider tach feedback gain?
	cmd = serr;

	cmd = max(cmd, -CMD_MAX);
	cmd = min(cmd, CMD_MAX);

	cmd = cmd + CMD_MIDRANGE;

//  y = amp + amp*sin(6.28*freq*ticks*0.005);
  P1.writeAnalog(CMD_COUNTS(cmd), 3, 2); //writes analog data to P1 output module

  ticks++;

#if (TIMER_INTERRUPT_DEBUG > 0)
    static uint32_t curMillis = 0;
      
    curMillis = millis();
    
    if (curMillis > TIMER_INTERVAL_MS)
    {
      Serial.print("ITimer: millis() = "); Serial.print(curMillis);
      Serial.print(", delta = "); Serial.println(curMillis - preMillisTimer);
    }
    
    preMillisTimer = curMillis;
#endif

  //timer interrupt toggles LED_BUILTIN
  digitalWrite(PIN_A1,toggle);
  toggle = !toggle;
}

void setup()
{

   while (!P1.init()){ 
    ; //Wait for Modules to Sign on   
    //GCLK->GENCTRL
  }

  // Configure A2D
  Serial.print(F("\nP1-04AD Config:")); Serial.println(P1.configureModule(P1_04AD_CONFIG, P104AD_MODULE_NUM));
  
  gpio_config();  

  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  
  delay(100);

  Serial.print(F("\nStarting TimerInterruptTest on ")); Serial.println(BOARD_NAME);
  Serial.println(SAMD_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

  // Interval in millisecs
  if (ITimer.attachInterruptInterval_MS(TIMER_INTERVAL_MS, TimerHandler))
  {
    preMillisTimer = millis();
    Serial.print(F("Starting ITimer OK, millis() = ")); Serial.println(preMillisTimer);
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  // Initialize controller
  lvdtVin[0] = LVDT_VOLTS(P1.readAnalog(1,LVDT_VIN_CHANNEL));

	Kp = Kpmax;  	
	Ki = bz*Kp;  // Kp / Ti;


}

void loop()
{
  static unsigned long lastTimer   = 0; 
  static bool timerStopped         = false;
  static bool ledToggle            = true;
  char rx_byte = 0;

  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character
    
    Serial.print("rx_byte:"); Serial.println(rx_byte);
  } 

  digitalWrite(LED_BUILTIN, ledToggle);

  Serial.print(F("Vlvdt:")); Serial.println(lvdtVin[1]);
  Serial.print(F("dVlvdt:")); Serial.println(ddtLvdt);
  Serial.print(F("Vedge:")); Serial.println(edgeVin);
  Serial.print(F("cmd:")); Serial.println(cmd);
  
  ledToggle = !ledToggle;
  delay(1000);
}

void gpio_config(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_A1, OUTPUT);
}
