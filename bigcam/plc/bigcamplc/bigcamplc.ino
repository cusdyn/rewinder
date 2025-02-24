/*
  River Valley Converting Rewinder web Wedge Control
  BIGCAM Retrofit
  Custom Dynamics, LLC
  Mike Timmons
  August 2024

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
#include <string.h>

/* SLOT
   _____  __1__ __2__ __3__ __4__ __5__
	|  P  ||  P  |  P  |  P  |  P  |  P  |
	|  1  ||  1  |  1  |  1  |  1  |  1  |
	|  A  ||  0  |  0  |  0  |  1  |  1  |
	|  M  ||  4  |  2  |  4  |  5  |  5  |
	|  -  ||  A  |  H  |  D  |  D  |  C  |
	|  1  ||  D  |  S  |  A  |  D  |  D  |
	|  0  ||     |  C  |  L  |  1  |  D  |
	|  0  ||     |     |  2  |     |  1  |
	 ¯¯¯¯¯  ¯¯¯¯¯ ¯¯¯¯¯ ¯¯¯¯¯ ¯¯¯¯¯ ¯¯¯¯¯

*/

// TC3, TC4, TC5 max permissible TIMER_INTERVAL_MS is 1398.101 ms, larger will overflow, therefore not permitted
// Use TCC, TCC1, TCC2 for longer TIMER_INTERVAL_MS
#define SAMPLE_RATE              (100.0)
#define TIMER_INTERVAL_MS        (1000/SAMPLE_RATE)

#define DURATION     20  // seconds
#define RECORDLENGTH (int)(SAMPLE_RATE*DURATION)


#ifndef LED_BUILTIN
  #define LED_BUILTIN       13
#endif

#define SD_CHIP_SELECT 28

#define P104AD_MODULE_NUM 1
#define COUNT_RANGE 65535

#define DAC_SLOT  3
#define DAC_CMD_OUT_CHANNEL  2
#define DAC_ACTIVE_OUT_CHAN  3
#define DAC_INACTIVE_OUT_CHAN  4


#define LVDT_VIN_CHANNEL 1
#define EDGE_VIN_CHANNEL 2
#define EDGE_IIN_CHANNEL 4

#define CMD_MAX  5.0
#define CMD_MIDRANGE 5.0


#define CMD_VRANGE     10



#define CMD_COUNTS(x)   (x*4095.0/(float)CMD_VRANGE)
#define DENC(x1,x2)     ((x2-x1)*SAMPLE_RATE)

#define INNER_LOOP_GAIN 1

#define DIO_SLOT 4
#define HOLD_SWITCH_BIT 0x01
#define HOLD_RIGHT_BIT  0x02
#define HOLD_LEFT_BIT   0x04


#define ENC_COUNTS_PER_INCH   16216

#define BANNER_MIDCOUNT       39321

/* 
Transport Phase lag given by  TL = -57.3*w*Td

specifify a maximum permissible TL=-30 degrees for a delay Td and arrive at a targetr crossover wc.

Td = time delay from input roll tangent to Edge Guide sensor location.
Td = L/Speed where L is path length from tangent exit to Edge Guide.
Speed is Web speed measured from reflective sensor on input idler pulley.

Speed = IdlerCircumference(C)/RotationPeriodMeasured(T)

wc = (30/57.3)/Td =  (30/57.3)*Speed/L = (30/57.3)*(C/T)/L 

For C = 0.6597 meter idler circumference (8.25" dia) and
    L = 1.5 meter path length  (5 feet)

    (30/57.3)*(C/T)/L = (30/57.3)*(0.6597/T)/1.5 = 0.2303   = WC_DES_FACTOR

    so...

    wc = WC_DES_FACTOR/T    where again T is the idler rotation period.

*/
#define WC_DES_FACTOR            (0.2303) 
#define MAX_IDLER_DRUM_PERIOD    (3.0)

#define HOLD_VDRIVE_MAG (2.0)
// private functions
static void LogToFile(bool print, int counter);
static void sd_init(void);

// Control loop Timer configuration
volatile uint32_t preMillisTimer = 0;
int   ticks=0;

// numerical differentiation of LVDT
float ddtEnc = 0;


/*
    Compensator zero is set for a desired phase hump of 85 degrees.
    This is unconventional, but it's to permit 30 degrees phase loss due to transport lag,
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
float Kpe   = .2922;   // Ke/Kb = encoder/edge = [2500*60/(9.25*0.0254)]/[2185*1000]
float bz    = 0.001;   // zero location
float Kpmax = .0001;  //23;   // max proportional gain regardless of web speed 
float wcmax = 2;  //10;     // open-loop crossover for Kpmax   11

// variable gains
float wcdes;  

float tdelay = 0;        // no estimate for web speed until idler period measured
float Kp=0;              // loop will calculate gains
float Ki=0;
float Kin=INNER_LOOP_GAIN;

// control variables
float perr=0.0;      // position error: outer loop
float serr=0.0;      // speed error: inner loop reference
float u=0.0;         // proportional action
float ui=0.0;        // inegral control action
float uih=0.0;        // inegral control action
float udh=0.0;        // inegral control action
float cmd=0.0;       // signed command
float vcmd=0.0;      // voltage command out to VFD

// Speed circuit
// Create HSC class object for slot 2. 
// It also automatically creates 2 P1_HSC_CHANNEL objects for this slot
P1_HSC_Module HSC(2); 


int            lastSpeedCounts   = 0;
int            speedCounts       = 0;

int   encoderCounts[2];
int   encoderStart       = 0;
int   holdPosRef         = 0;
float holdVdrive         = 0.0;

int   speed_ticks  = 0;
float period=MAX_IDLER_DRUM_PERIOD;   // intialize slow
bool  pulseAction=false;


int edgeCountsIn = 0;

/* generated using tool at https://facts-engineering.github.io/modules/P1-04AD/P1-04AD.html
 *  Channel 1=+-10V    - SIM mode for voltage the represent encoder
 *  Channel 2=0-10V    - SIM mode for edge sensor
 *  Channel 3=0-10V
 *  Channel 4=0-20mA   - production mode: Banner 4-20mA edge sensor input
 */
const char P1_04AD_CONFIG[] = { 0x40, 0x03, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x21, 0x01, 0x00, 0x00, 0x22, 0x01, 0x00, 0x00, 0x23, 0x03 };

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

unsigned int loopTick=0;

#define LOG_BUFF_LEN 150
char logBuffer[LOG_BUFF_LEN];

static unsigned long lastTimer   = 0; 
static bool timerStopped         = false;
static bool ledToggle            = true;

// hold switch logic
bool too_slow = true;

// hold switch
uint8_t  hold_switch=0;
bool     holding=false;
bool     force_hold=false;

// sys ID parameters
struct {
  uint8_t  active;
  int      ampid;
  int      freqid;
} sysId;

float sysIdFreq[] = {0.0125, 0.025, 0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2, 0.225, 0.25, 0.275, 0.3, 0.325, 0.35, 0.375, 0.4, 1.0 }; 
float sysIdAmp[]  = {1.0,3.0,5.0};

float kphold[]    = {0.0, 0.0015,0.002, 0.0021,0.0022,0.0023,0.0024,0.0025,0.0026,0.0027,0.0028,0.0029,0.003};


int       encin[RECORDLENGTH];
int       banner[RECORDLENGTH];
int dataoutcnt=0;
int bannercnt=0;

bool logbanner = false;

inline void process_hold_switch()
{
  hold_switch = P1.readDiscrete(DIO_SLOT);   // REPLACE this with Discrete input module read
  if(((hold_switch & HOLD_SWITCH_BIT) == HOLD_SWITCH_BIT) && (holding==false))
  {
    digitalWrite(PIN_A2,1);
    holding = true;
    

    holdPosRef = encoderCounts[1] + 2500*4;

    // kill the integrator
    ui = 0;
  }
  else if (((hold_switch & HOLD_SWITCH_BIT) != HOLD_SWITCH_BIT) && (holding==true))
  {
    digitalWrite(PIN_A2,0);
    holding = false;

    // kill the integrator
    ui = 0;
  }

  // process jog button
  holdVdrive = 0.0;
  if( holding == true )
  {
    if((hold_switch & HOLD_LEFT_BIT) == HOLD_LEFT_BIT)
    {
      holdVdrive = HOLD_VDRIVE_MAG;
    }
    else if((hold_switch & HOLD_RIGHT_BIT) == HOLD_RIGHT_BIT)
    {
      holdVdrive = -HOLD_VDRIVE_MAG; 
    }
    else
    {
      // if in hold mode but no button down, latch the reference pos
      holdPosRef = encoderCounts[1];
    }
  }
}


#define SIM_EDGE_V2COUNTS   7502    // counts per volt

void TimerHandler()
{
  //timer interrupt pin toggle
  digitalWrite(PIN_A1,1);

  encoderCounts[1]      = HSC.CNT2.readPosition() - encoderStart;

  edgeCountsIn = P1.readAnalog(1,EDGE_IIN_CHANNEL) - BANNER_MIDCOUNT;

  // rack rate: numerical differentiation
  ddtEnc = DENC(encoderCounts[0], encoderCounts[1]);
  encoderCounts[0] = encoderCounts[1];  // propagate LVDT state

  if( logbanner == true )
  {
    if( (ticks % 4) == 0 )
    {
      if (bannercnt < RECORDLENGTH)
      {
        banner[bannercnt]  = edgeCountsIn;
        bannercnt++;
      }
    }
  }


  process_hold_switch();

  if( sysId.active == true )
  {
    cmd = sysIdAmp[sysId.ampid] * sin(2 * M_PI * sysIdFreq[sysId.freqid] * ticks / SAMPLE_RATE);

    if( (ticks % 4) == 0 )
    {
      if (dataoutcnt < RECORDLENGTH)
      {
        encin[dataoutcnt]  = encoderCounts[1];
        dataoutcnt++;
      }
    }
  }
  else if( (holding == true) || (force_hold == true) )
  {
    // Close loop on encoder

    // close loop on rack only. no EdgeGuide

    cmd = holdVdrive;

  }
  else  // closed loop
  {

   
    // position error from banner edge mapped to encoder space
    // positive sign because drive sign is opposit encoder and banner sensor sign
    perr = edgeCountsIn*Kpe;   // map banner counts to encoder counts

    // P term
    u = Kp*perr;

    // I term: integrator on outer loop handles Edge Guide saturation:
    // it integrated back to linear output region around the null point.
    ui = ui + (Ki*perr)/SAMPLE_RATE;

    // clamp integrator
    ui = max(ui, -CMD_MAX);
    ui = min(ui, CMD_MAX);

    // speed error: inner loop reference
    serr = (u + ui) - ddtEnc/8000;

    // unity gain on inner loop so command output to valve amp is just this...
    cmd = Kin*serr;

  }

  // clamp to +- maximum command
  cmd = max(cmd, -CMD_MAX);
	cmd = min(cmd, CMD_MAX);   

  // offset for 0-10 V wandfluh valve amp input. the amp will map 5-0V one way and 5-10V the other.
	vcmd = cmd + CMD_MIDRANGE;
  //vcmd = CMD_MIDRANGE;

  P1.writeAnalog(CMD_COUNTS(vcmd), DAC_SLOT, DAC_CMD_OUT_CHANNEL); //writes analog data to P1 output module

  ticks++;



  // read input side web idler pulse counter
  speedCounts      = HSC.CNT1.readPosition();
  

  speed_ticks++; // increment period counter
  
  if( speedCounts == (lastSpeedCounts+1))
  {
    // completed a rotation
    lastSpeedCounts = speedCounts;
    
    period = float(speed_ticks)/SAMPLE_RATE;


    wcdes = WC_DES_FACTOR/period;
    speed_ticks=0;  

    pulseAction=true;
  }
  else if (speedCounts > (lastSpeedCounts+1))
  {
    // extra ticks so reset
    speed_ticks=0;
    lastSpeedCounts = speedCounts;
  }

  // latch low period in case no pulses coming through...
  if(pulseAction = true)
  {
    if(float(speed_ticks)/SAMPLE_RATE > MAX_IDLER_DRUM_PERIOD)
    {
      // kill the gain
      period = MAX_IDLER_DRUM_PERIOD;
      pulseAction = false;
    }
    
  }





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

 #if 0 
  if(period < MAX_IDLER_DRUM_PERIOD)
  {
    if(too_slow==true)
    {
        // one time write on state change
        P1.writeAnalog(CMD_COUNTS(10.0), DAC_SLOT, DAC_ACTIVE_OUT_CHAN);   // green panel LED on
        P1.writeAnalog(CMD_COUNTS(0.0), DAC_SLOT, DAC_INACTIVE_OUT_CHAN);  // green off
        too_slow = false;
    }

  
	  Kp = min(Kpmax, Kpmax * pow(10, log10(wcdes / wcmax)));
   	Ki = bz*Kp;  // Kp / Ti;
    Kin = INNER_LOOP_GAIN;
  }
  else  // too slow
  {
    if(too_slow==false)
    {
        // one time write on state change
        P1.writeAnalog(CMD_COUNTS(10.0), DAC_SLOT, DAC_INACTIVE_OUT_CHAN);   // red panel LED on
        P1.writeAnalog(CMD_COUNTS(0.0), DAC_SLOT, DAC_ACTIVE_OUT_CHAN);  // green off
        too_slow = true;
    }

    // Kill the gain and the integrator
    Kp  = 0;
    Ki  = 0;
    Kin = 0;
    ui  = 0;
  }
#else
  // test no gain scaling for speed.
  Kp = Kpmax;
  Ki = bz*Kp;  // Kp / Ti;
#endif

  digitalWrite(PIN_A1,0);


	//			printf("Counter: %d period:%4.2f\n", counter[1], period );

}

#define BC_STR_LEN 20
#define NUM_BOOT_CYCLE_LOGS 10
#define FILENAME_LEN 20
char logFileName[FILENAME_LEN];

#define BUFSZ 40
typedef struct CBUF
{
  char buf[BUFSZ];
  int in;
  int out;
};

CBUF cb;

void setup()
{
  File bcFile;
  char bcString[BC_STR_LEN];
  int bcscnt=0;

  memset(cb.buf, 0, BUFSZ);
  cb.in = 0;
  cb.out = 0;

   while (!P1.init()){ 
    ; //Wait for Modules to Sign on   
    //GCLK->GENCTRL
  }

  // set valve command to midrange (noflow)
  P1.writeAnalog(CMD_COUNTS(CMD_MIDRANGE), DAC_SLOT, DAC_CMD_OUT_CHANNEL);

  // Configure A2D
  Serial.print(F("\nP1-04AD Config:")); Serial.println(P1.configureModule(P1_04AD_CONFIG, P104AD_MODULE_NUM));

  gpio_config();  

  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  
  sd_init();

  delay(100);

  
	Kp = Kpmax;  	
	Ki = bz*Kp;  // Kp / Ti;

  // intialize web speed pulse counter
  HSC.CNT1.isRotary = false;
  HSC.CNT1.enableZReset = false;
  HSC.CNT1.inhibitOn = false; //oneZ, threeIn, twoZ, fourIn
  HSC.CNT1.mode = stepDirection;  //quad4x, quad1x
  HSC.CNT1.polarity = positiveDirection;  //negativeDirection

  // Initialize Encoder input
  HSC.CNT2.isRotary = true;
  HSC.CNT2.enableZReset = false;
  HSC.CNT2.inhibitOn = false; //oneZ, threeIn, twoZ, fourIn
  HSC.CNT2.mode = stepDirection;  //quad4x, quad1x
  HSC.CNT2.polarity = positiveDirection;  //negativeDirection

  HSC.configureChannels();  //Load settings into HSC module. Leave argument empty to use default CNT1 and CNT2 

  // Initialize rotary encoder
  HSC.CNT2.setPosition(1000000);


    // default the status LEDs
  P1.writeAnalog(CMD_COUNTS(10.0), DAC_SLOT, DAC_INACTIVE_OUT_CHAN);   // red panel LED on
  P1.writeAnalog(CMD_COUNTS(0.0), DAC_SLOT, DAC_ACTIVE_OUT_CHAN);  // green off


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
  memset(logFileName,0,FILENAME_LEN);
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


  // initialize controller
  encoderStart = HSC.CNT2.readPosition();
  
  
  // don't know if this is needed
  delay(100);
}

char tmpbuffer[40];

void loop()
{
  int index; 

  digitalWrite(LED_BUILTIN, ledToggle);
  ledToggle = !ledToggle;

  LogToFile(true, ticks);


  loopTick++;

  #define MAX_NUM_PARAMS  5
  #define MAX_PARAM_LEN       10
  char scmd[MAX_NUM_PARAMS][MAX_PARAM_LEN];

  while (Serial.available() > 0) 
  {
    int ch = Serial.read();

    cb.buf[cb.in++] = (char)ch;
    if(cb.in == BUFSZ)
    {
      memset(cb.buf, 0, BUFSZ);
      cb.in=0;
    }
    if( ch == '\n')
    {
      char *token;

      // process command string
      Serial.println(cb.buf);

      /* get the first token */
      token = strtok(cb.buf, ",");
   
      /* walk through other tokens */
      int tcnt = 0;
      if( strlen(token) < MAX_PARAM_LEN)
      {
        sprintf(scmd[tcnt], "%s",token);
        while( token != NULL ) {
            if( tcnt < MAX_NUM_PARAMS )
            {
              Serial.println( scmd[tcnt++] );
              token = strtok(NULL, ",");
              if( strlen(token) < MAX_PARAM_LEN)
              {
                sprintf(scmd[tcnt], "%s",token);
              }
            }
            else
            {
              Serial.println( "Too Many Parameters" );              
              break;
            }
            
        }
      }
      else
      {
        Serial.println( "Parameter Error" );
      }

      Serial.println( "Process command" );
      switch( scmd[0][0] )
      {
        case 'q':
          Serial.println( "quit" );
          sysId.active = false;
          sysIdLog();
          force_hold = false;
          uih=0;  // kill the integrator
          break;
        case 's':
          Serial.println( "Sys ID" );
          sscanf(&scmd[1][0], "%d", &sysId.freqid);
          sscanf(&scmd[2][0], "%d", &sysId.ampid);       
          sysId.active = true;
          dataoutcnt=0;

          sprintf(tmpbuffer,"S:%s,%s, %d:%d %5.1f:%5.1f",
                  scmd[1],scmd[2],sysId.freqid, sysId.ampid, sysIdFreq[sysId.freqid], sysIdAmp[sysId.ampid]);
          Serial.println(tmpbuffer);          
          break;
        case 'h':
          force_hold=true;
          break;
        case 'r':
          sscanf(&scmd[1][0], "%d", &holdPosRef);
          break;
        case 'g':
          P1.writeDiscrete(1,DIO_SLOT,1);
          break;
        case 'n':
          P1.writeDiscrete(0,DIO_SLOT,1);
          break;  
        case 'b':
          // banner log
          bannercnt=0;
          logbanner=true;
          break;
        case 'v':
          logbanner=false;
          bannerSensorLog();
          break;
        case 'w':
          cmd = 5.0;
          break;
        case 'e':
          cmd = -5.0;
          break; 
        case 'f':
          cmd = 0;
          break;
        case 'k':
          sscanf(&scmd[1][0], "%d", &index);
          Kp = kphold[index];
          break;                     
        default:
          Serial.println( "Other" );
          break;
      }

      // reset the buffer
      cb.in=0;
      memset(cb.buf, 0, BUFSZ);
    }
  }


  delay(1000);
}


void gpio_config(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_A1, OUTPUT);
  pinMode(PIN_A2, OUTPUT);

}



static void bannerSensorLog(void)
{
  int i;
  char sample[20];
  File dataFile = SD.open("banner.txt", FILE_WRITE);
  
  if (dataFile)
  {
    for(i=0; i<bannercnt; i++)
    {
      sprintf(sample, "%d", banner[i]);
      dataFile.println(sample);
    }

    dataFile.close();
  }

}


static void sysIdLog(void)
{
  int i;
  char filename[20];
  sprintf(filename, "s%d%d.txt", sysId.freqid, sysId.ampid);

  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile)
  {
    char sample[20];
    dataFile.println(sysIdAmp[sysId.ampid],6);
    dataFile.println(sysIdFreq[sysId.freqid],6);


    for(i=0; i<dataoutcnt; i++)
    {
      sprintf(sample, "%d", encin[i]);
      dataFile.println(sample);
    }
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    // try to initialize the SD card again...
    sd_init();
  }


}

static void LogToFile(bool print, int counter)
{
//  memset(logBuffer,0,LOG_BUFF_LEN);
  sprintf(logBuffer, 
          "%d, %d, %d, %6.1f, %4.2f, %4.4f, %4.4f, %4.2f,%4.2f,%5.3f,%4.2f,%4.2f, %6.5f, %d, %4.2f, %d",
           counter, edgeCountsIn, encoderCounts[1], ddtEnc, perr, u, ui, cmd, vcmd, period, wcdes, Kp, speedCounts, hold_switch, holdPosRef);

 #if 1
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(logFileName, FILE_WRITE);
  // if the file is available, write to it:
  
  if (dataFile) {
    dataFile.println(logBuffer);
    dataFile.close();

    if(print){
      Serial.println(logBuffer);
    }
  }
  // if the file isn't open, pop up an error:
  else {
    if(print){
      Serial.println("error opening datalog.txt");
    }

    // try to initialize the SD card again...
    sd_init();
  }
#else
  Serial.println(logBuffer);
#endif

}



/*
  Initialize SD card
*/
static void sd_init(void)
{
// Initialize SD counter
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (SD.begin(SD_CHIP_SELECT)) {
    Serial.println("card initialized.");
  }
  else
  {
    Serial.println("Card failed, or not present");
  }
}