/*
  River Valley Converting Rewinder web Wedge Control
  Custom Dynamics, LLC
  Mike Timmons
  August 2023

*/

// Select only one to be true for SAMD21. Must must be placed at the beginning before #include "SAMDTimerInterrupt.h"
#define USING_TIMER_TC3         true      // Only TC3 can be used for SAMD51

/////////////////////////////////////////////////////////////////

#include "SAMDTimerInterrupt.h"
#include <P1AM.h>
#include <P1_HSC.h>
#include <LibPrintf.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

/* SLOT
   _____  __1__ __2__ __3__
	|  P  ||  P  |  P  |  P  |
	|  1  ||  1  |  1  |  1  |
	|  A  ||  0  |  0  |  0  |
	|  M  ||  4  |  2  |  4  |
	|  -  ||  A  |  H  |  D  |
	|  1  ||  D  |  S  |  A  |
	|  0  ||     |  C  |  L  |
	|  0  ||     |     |  2  |   
	 ¯¯¯¯¯  ¯¯¯¯¯ ¯¯¯¯¯ ¯¯¯¯¯
*/

// TC3, TC4, TC5 max permissible TIMER_INTERVAL_MS is 1398.101 ms, larger will overflow, therefore not permitted
// Use TCC, TCC1, TCC2 for longer TIMER_INTERVAL_MS
#define SAMPLE_RATE              (100.0)
#define TIMER_INTERVAL_MS        (1000/SAMPLE_RATE)


#ifndef LED_BUILTIN
  #define LED_BUILTIN       13
#endif

#define SD_CHIP_SELECT 28

#define P104AD_MODULE_NUM 1
#define COUNT_RANGE 65535

#define DAC_SLOT  3
#define DAC_CMD_OUT_CHANNEL  2
#define DAC_POT_OUT_CHANNEL  1


#define LVDT_VIN_CHANNEL 1
#define EDGE_VIN_CHANNEL 2
#define POT_VIN_CHANNEL  3

#define CMD_MAX  5.0
#define CMD_MIDRANGE 5.0

#define LVDT_VRANGE    10
#define EDGE_VRANGE    20
#define CMD_VRANGE     10

#define LVDT_VOLTS(x)  (LVDT_VRANGE*x/COUNT_RANGE)
#define EDGE_VOLTS(x)  (EDGE_VRANGE*x/COUNT_RANGE)
#define CMD_COUNTS(x)   (x*4095/CMD_VRANGE)
#define DLVDT(x1,x2)     ((x2-x1)*SAMPLE_RATE)



/* 
Transport Phase lag given by  TL = -57.3*w*Td

specifify a maximum permissible TL=-30 degrees for a delay Td and arrive at a targetr crossover wc.

Td = time delay from input roll tangent to Edge Guide sensor location.
Td = L/Speed where L is path length from tangent exit to Edge Guide.
Speed is Web speed measured from reflective sensor on input idler pulley.

Speed = IdlerCircumference(C)/RotationPeriodMeasured(T)

wc = (30/57.3)/Td =  (30/57.3)*Speed/L = (30/57.3)*(C/T)/L 

For C = 0.75 meter idler circumference and
    L = 1.5 meter path length

    (30/57.3)*(C/T)/L = (30/57.3)*(0.75/T)/1.5 = 0.2618   = WC_DES_FACTOR

    so...

    wc = WC_DES_FACTOR/T    where again T is the idler rotation period.

*/
#define WC_DES_FACTOR            (0.2618) 

// Control loop Timer configuration
volatile uint32_t preMillisTimer = 0;
int   ticks=0;

// Sensor sample inputs
float lvdtVin[2];
float edgeVin = 0.0;

// numerical differentiation of LVDT
float ddtLvdt = 0;


/*
    Compensator zero is set for a desired phase hump of 85 degrees.
    This is unconventional, but it's to permit 30 degrees pahse loss due to transport lag,
    or a design phase margin of 50 degrees intended.

    % Ignoring transport lag consider you're designing for a phase margin of 
    % 85 degrees.
    dpm = 85*pi/180; % desired phase bump
    
    % resultant alpha
    alpha = (1/sin(dpm) - 1)/(1 + 1/sin(dpm))
    Kt = 1;  % tach feedback gain

    % compensator zero
    b = alpha*(a+Kt*Kv);

    bz = b 
*/
// Control parameters  ... see outerpi.m
float Kpe   = .0254;   // Kl/Keg = (80 V/meter)/(3149.6 V/M) scales edge guide to LVDT 
float bz    = 0.0858;   // zero location
float Kpmax = 23;    // max proportional gain regardless of web speed
float wcmax = 10;     // open-loop crossover for Kpmax

// variable gains
float wcdes = wcmax/20;  // initialize desired wc for low gain and don't set it until speed is non-zero
float tdelay = 0;        // no estimate for web speed until idler period measured
float Kp=0;              // loop will calculate gains
float Ki=0;

// control variables
float perr=0.0;      // position error: outer loop
float serr=0.0;      // speed error: inner loop reference
float u=0.0;         // proportional action
float ui=0.0;        // inegral control action
float cmd=0.0;       // command output to solenoid valve amplifier


// Speed circuit
// Create HSC class object for slot 2. 
// It also automatically creates 2 P1_HSC_CHANNEL objects for this slot
P1_HSC_Module HSC(2); 

bool  printCounts = false;
int   lastCounts   = 0;
int   counts       = 0;
int   speed_ticks  = 0;
float period=0;

/* generated using tool at https://facts-engineering.github.io/modules/P1-04AD/P1-04AD.html
 *  Channel 1=0-10V
 *  Channel 2=+-10V
 *  Channel 3=0-10V
 *  Channel 4=+-10V
 */
const char P1_04AD_CONFIG[] = { 0x40, 0x03, 0x00, 0x00, 0x20, 0x01, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0x22, 0x01, 0x00, 0x00, 0x23, 0x03 };

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

void TimerHandler()
{
  static bool toggle = false;

  // Sensor input
  lvdtVin[1] = LVDT_VOLTS(P1.readAnalog(1,LVDT_VIN_CHANNEL));
  edgeVin    = EDGE_VOLTS(P1.readAnalog(1,EDGE_VIN_CHANNEL));

  // LVDT rate: numerical differentiation
  ddtLvdt = DLVDT(lvdtVin[0],lvdtVin[1]);
  lvdtVin[0] = lvdtVin[1];  // propagate LVDT state

  // position error from edge guide in LVDT space
  perr = -edgeVin*Kpe;   // map edge guide voltage to LVDT equivalent

	// P term
	u = Kp*perr;

	// I term: integrator on outer loop handles Edge Guide saturation:
  // it integrated back to linear output region around the null point.
	ui = ui + (Ki*perr)/SAMPLE_RATE;

  // clamp integrator
  ui = max(ui, -CMD_MAX);
  ui = min(ui, CMD_MAX);

  // speed error: inner loop reference
	serr = (u + ui) - ddtLvdt;

	// unity gain on inner loop so command output to valve amp is just this...
	cmd = serr;

  // clamp to +- maximum command
  cmd = max(cmd, -CMD_MAX);
	cmd = min(cmd, CMD_MAX);   

  // offset for 0-10 V wandfluh valve amp input. the amp will map 5-0V one way and 5-10V the other.
	cmd = cmd + CMD_MIDRANGE;

  
  P1.writeAnalog(CMD_COUNTS(cmd), DAC_SLOT, DAC_CMD_OUT_CHANNEL); //writes analog data to P1 output module

  ticks++;


  //timer interrupt pin toggle
  //digitalWrite(PIN_A1,toggle);
  //toggle = !toggle;

  // read input side web idler pulse counter
  counts = HSC.CNT1.readPosition();
  
  speed_ticks++; // increment period counter
  
  if( counts == (lastCounts+1))
  {
    // completed a rotation
    lastCounts = counts;
    printCounts = true;

    period = float(speed_ticks)/SAMPLE_RATE;


    wcdes = WC_DES_FACTOR/period;
    speed_ticks=0;  
  }
  else if (counts > (lastCounts+1))
  {
    // extra ticks so reset
    speed_ticks=0;
    lastCounts = counts;
  }
}

#define BC_STR_LEN 20
#define NUM_BOOT_CYCLE_LOGS 10
char logFileName[20];

void setup()
{
  File bcFile;
  char bcString[BC_STR_LEN];
  int bcscnt=0;
  

   while (!P1.init()){ 
    ; //Wait for Modules to Sign on   
    //GCLK->GENCTRL
  }

  // Configure A2D
  Serial.print(F("\nP1-04AD Config:")); Serial.println(P1.configureModule(P1_04AD_CONFIG, P104AD_MODULE_NUM));
  
  gpio_config();  

  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  
  // Initialize SD card
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

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

  // intialize web speed pulse counter
  HSC.CNT1.isRotary = false;
  HSC.CNT1.enableZReset = false;
  HSC.CNT1.inhibitOn = false; //oneZ, threeIn, twoZ, fourIn
  HSC.CNT1.mode = stepDirection;  //quad4x, quad1x
  HSC.CNT1.polarity = positiveDirection;  //negativeDirection

  HSC.configureChannels();  //Load settings into HSC module. Leave argument empty to use default CNT1 and CNT2 

  HSC.CNT1.setPosition(1000); //Initialise positions

  // BIAS the potentiometer dial
  P1.writeAnalog(CMD_COUNTS(10.0), DAC_SLOT, DAC_POT_OUT_CHANNEL);

  // log file management...
  bcFile = SD.open("bc.txt");

  // if the file opened okay, write to it:
  if (bcFile) {
    memset(bcString,0,BC_STR_LEN);
 
    while(bcFile.available() && (bcscnt < BC_STR_LEN)) 
    {
      bcString[bcscnt++] = bcFile.read();
    }
    sscanf(bcString,"%d", &bcscnt);

    Serial.print("entry boot count:"); Serial.print(bcscnt); Serial.println();
    bcFile.close();

    // remove the file    
    SD.remove("bc.txt");

    // remove an old log file
    if( bcscnt > NUM_BOOT_CYCLE_LOGS )
    {
      sprintf(bcString, "%d.txt", bcscnt-NUM_BOOT_CYCLE_LOGS);
      SD.remove(bcString);
    }
  }
  else
  {
    Serial.println("Failed to open bc.txt");
  }

  Serial.println("Updating boot count");
 
  bcscnt += 1;  // increment boot count;
  sprintf(bcString,"%d",bcscnt);
  sprintf(logFileName, "%d.txt", bcscnt);
  
  File bcOutFile = SD.open("bc.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (bcOutFile) {
   Serial.print("exit boot count:"); Serial.println(bcString);
    bcOutFile.println(bcString);
    // close the file:
    bcOutFile.close();
  }
  else
  {
    Serial.println("Failed to open bc.txt");
  }

  // don't know if this is needed
  delay(100);
}

unsigned int loopTick=0;
#define LOG_BUFF_LEN 50
char logBuffer[LOG_BUFF_LEN];
void loop()
{
  static unsigned long lastTimer   = 0; 
  static bool timerStopped         = false;
  static bool ledToggle            = true;
  char rx_byte = 0;
  float potScale;

  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character
    
    Serial.print("rx_byte:"); Serial.println(rx_byte);
  } 

  digitalWrite(LED_BUILTIN, ledToggle);

  //Serial.print(F("Vlvdt:")); Serial.println(lvdtVin[1]);
  //Serial.print(F("dVlvdt:")); Serial.println(ddtLvdt);
  //Serial.print(F("Vedge:")); Serial.println(edgeVin);
  //Serial.print(F("cmd:")); Serial.println(cmd);
  


  

  if( printCounts )
  {
    Serial.print("Counts : ");
    Serial.print(counts);
    Serial.print("Period : ");
    Serial.print(period);
    Serial.print("Wcdes: ");
    Serial.print(wcdes);
    Serial.println();   //print a blank line to look more organized

    printCounts=false;
  }


  ledToggle = !ledToggle;

  // Gain scale pot wiper nominal is 50% full scale = 1 multiplier.
  // 5-0V scales down 1 to zero. 5-10V scales multiplier  1 to 2
  potScale    = 2.0*P1.readAnalog(1,POT_VIN_CHANNEL)/(float)(COUNT_RANGE);

  /* SCALE GAIN Kp = Kpmax * 10 ^ (log10(wc / wmax))

    measured web speed gives estimate for delay time.
    From the web speed measurement we scaled our desired crossover
    to estimate taking 30 degrees of pahse margin from our generous
    phase hump offered by our compensator zero.

    Recall above we designed for 85 degrees of phase gain, and we
    selected wc where the web speed time delay takes 30 degrees for 
    a design phase margin target of 50 degrees.

    the plant model presumes a -20db/decade slope over the range of target
    bandwidth's scaled based on web speed.

    Therefore we slide KP on this -20Db slope line to set target crossover
    relative to design crossover wcmax set by design gain Kpmax.

  */

	Kp = potScale*min(Kpmax, Kpmax * pow(10, log10(wcdes / wcmax)));

  memset(logBuffer,0,LOG_BUFF_LEN);
  sprintf(logBuffer, 
          "%d,%5.3f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f",
           loopTick, edgeVin, lvdtVin[1], period, wcdes, Kp, potScale );

 
 // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(logFileName, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(logBuffer);
    dataFile.close();
    // print to the serial port too:
    Serial.println(logBuffer);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

  loopTick++;

  delay(1000);
}

void gpio_config(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_A1, OUTPUT);
}
