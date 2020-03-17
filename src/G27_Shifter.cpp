
//  G27 shifter pinout
//
//  DB9  Color            Description       Teensy
//  1    Purple           Clock             pin 19
//  2    Grey             Button Data       pin 17
//  3    Yellow           Button !CS & !PL  pin 18
//  4    Orange           Shifter X axis    pin 14 (A0)
//  5    White            GND               GND
//  6    Black            GND               GND
//  7    Red (Blue)       +5V               +5V   
//  8    Green            Shifter Y axis    pin 15 (A1)
//  9    Red              +5V               +5V

#include <Arduino.h>
#include <EEPROM.h>

// Teensy pin definitions
#define LED_PIN            13
#define DATA_IN_PIN        17
#define MODE_PIN           18
#define CLOCK_PIN          19
#define X_AXIS_PIN         14
#define Y_AXIS_PIN         15

// H-shifter mode analog axis thresholds
#define HS_XAXIS_12        450
#define HS_XAXIS_56        900
#define HS_YAXIS_135       1000
#define HS_YAXIS_246       400

// Digital inputs definitions
#define DI_REVERSE         1
#define DI_MODE            3
#define DI_RED_CENTERRIGHT 4
#define DI_RED_CENTERLEFT  5
#define DI_RED_RIGHT       6
#define DI_RED_LEFT        7
#define DI_BLACK_TOP       8
#define DI_BLACK_RIGHT     9
#define DI_BLACK_LEFT      10
#define DI_BLACK_BOTTOM    11
#define DI_DPAD_RIGHT      12
#define DI_DPAD_LEFT       13
#define DI_DPAD_BOTTOM     14
#define DI_DPAD_TOP        15

#define FLAGS_ADDRESS         0
#define HAT_MODE              0
#define POSITIVE_NEUTRAL_MODE 1

#define X_AXIS 0
#define Y_AXIS 1

typedef struct 
{
  bool isChangingMode;
  bool isActive;
  int button;
  byte flag;
} configMode;

byte flags;
configMode configModes[2];

bool isFlagSet(byte flags, byte flag)
{
  return (flags & flag) == flag;
}

byte setFlag(byte flags, byte flag)
{
  return flags | flag;
}

byte unsetFlag(byte flags, byte flag)
{
  return flags & ~flag;
}

byte toggleFlag(byte flags, byte flag)
{
  return flags ^ flag;
}

configMode newConfigMode(byte flags, int flagBit, int button)
{
 configMode cm;
 cm.flag= 1 << flagBit;
 cm.button=button;
 cm.isChangingMode=false;
 cm.isActive= isFlagSet(flags, cm.flag);

 return cm;
}

int hatMap[11]={-1,90,270,-1,180,135,225,-1,0,45,315};
void setHat(bool *hatButtons)
{
  int hatPos = 0;
  for(int i=0; i<4; i++)
    {
      hatPos += hatButtons[i]<<i;
      Joystick.button(i+19,LOW);
    }
  if (hatPos > 10) hatPos = 0;
  Joystick.hat(hatMap[hatPos]);
}

void updateButtonStates()
{
  digitalWrite(MODE_PIN, LOW);         // Parallel mode: inputs are read into shift register
  delayMicroseconds(10);               // Wait for signal to settle
  digitalWrite(MODE_PIN, HIGH);        // Serial mode: data bits are output on clock falling edge
}

bool getNextButtonState()
{
  digitalWrite(CLOCK_PIN, LOW);      // Generate clock falling edge
  delayMicroseconds(10);             // Wait for signal to settle

  int b =digitalRead(DATA_IN_PIN);   // Read data bit and store it into bit array

  digitalWrite(CLOCK_PIN, HIGH);     // Generate clock rising edge
  delayMicroseconds(10);
  return b==1;    
}

int getRawGear()
{
  //Read shifter position
  int x=analogRead(X_AXIS);
  int y=analogRead(Y_AXIS);  

  int gy = y>HS_YAXIS_135 ? 1:
           y<HS_YAXIS_246 ? 2:
           0;
  int gx = gy == 0 ? 0:
           x<HS_XAXIS_12 ? 0:
           x>HS_XAXIS_56 ? 4:
           2;

  return gy + gx;                  
}

// Called at startup
// Must initialize hardware and software modules
void setup()
{
  // G25 shifter analog inputs configuration 
  pinMode(X_AXIS_PIN, INPUT_PULLUP);   // X axis
  pinMode(Y_AXIS_PIN, INPUT_PULLUP);   // Y axis
  
  // G25 shift register interface configuration 
  pinMode(DATA_IN_PIN, INPUT);         // Data in
  pinMode(MODE_PIN, OUTPUT);           // Parallel/serial mode
  pinMode(CLOCK_PIN, OUTPUT);          // Clock
  
  // LED output mode configuration 
  pinMode(LED_PIN, OUTPUT);            // LED
  
  // Virtual joystick configuration 
  Joystick.useManualSend(true);        // Joystick output is synchronized
  
  // Virtual serial interface configuration 
  Serial.begin(38400);
  
  // Virtual joystick initialization 
  Joystick.X(0);
  Joystick.Y(0);
  Joystick.Z(0);
  Joystick.Zrotate(0);
  Joystick.sliderLeft(0);
  Joystick.sliderRight(0);
  Joystick.hat(-1);

  // Digital outputs initialization
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MODE_PIN, HIGH);
  digitalWrite(CLOCK_PIN, HIGH); 
  flags = EEPROM.read(FLAGS_ADDRESS);

  configModes[HAT_MODE] = newConfigMode(flags, HAT_MODE, DI_DPAD_TOP);
  configModes[POSITIVE_NEUTRAL_MODE] = 
      newConfigMode(flags, POSITIVE_NEUTRAL_MODE, DI_DPAD_BOTTOM);
  
}

// Called in a loop after initialization
void loop() 
{
  // Reading of button states from G27 shift register
  updateButtonStates();
  // digitalWrite(MODE_PIN, LOW);         // Parallel mode: inputs are read into shift register
  // delayMicroseconds(10);               // Wait for signal to settle
  // digitalWrite(MODE_PIN, HIGH);        // Serial mode: data bits are output on clock falling edge
  
  bool b[16];
  for(int i=0; i<16; i++)              // Iteration over both 8 bit registers
  {
    // digitalWrite(CLOCK_PIN, LOW);      // Generate clock falling edge
    // delayMicroseconds(10);             // Wait for signal to settle
  
    // b[i]=digitalRead(DATA_IN_PIN);     // Read data bit and store it into bit array
    
    // digitalWrite(CLOCK_PIN, HIGH);     // Generate clock rising edge
    // delayMicroseconds(10);             // Wait for signal to settle
    b[i]=getNextButtonState();
  }

  int gear = getRawGear();                  
  
  bool isReverse = (gear==6 && b[DI_REVERSE]==1);

  if (isReverse) gear=0;
  else b[DI_REVERSE] = 0;

  for(int i = 1; i < 7; i++)
  {
    Joystick.button(i, (!isReverse && gear==i) );
  }
  Joystick.button(24,(configModes[POSITIVE_NEUTRAL_MODE].isActive && !isReverse && gear==0));

  int topButton = 16;
  if (configModes[HAT_MODE].isActive)
  {
    setHat(&b[12]);
    topButton = 12;
  } 
  else
  {
    Joystick.hat(-1);   
  }
  // Set state of virtual buttons for all the physical buttons (including reverse)
  for(int i=0; i<topButton; i++) Joystick.button(7+i, b[i]);

  // Write new virtual joystick state
  Joystick.send_now();
  
  // Write inputs and outputs (remove comments to debug)
  /*
  Serial.print(" X axis: ");
  Serial.print(x);
  Serial.print(" Y axis: ");
  Serial.print(y);
  Serial.print(" Digital inputs: ");
  for(int i=0; i<16; i++)Serial.print(b[i]);
  Serial.print(" ");
  Serial.print(" Gear: ");
  Serial.print(gear);
  Serial.print(" Mode: ");
  Serial.print(mode);
  Serial.print(" Shift: ");
  Serial.println(shift);
  */
  
  bool areFlagsInvalid = false;
  if (isReverse && b[4] && b[5] && b[6] && b[7])
  {
    for (int i=0; i<2; i++)
    {
      if (!configModes[i].isChangingMode && b[configModes[i].button])
      {
        configModes[i].isChangingMode=true;
        Joystick.button(32,HIGH);
        configModes[i].isActive ^= true;
        flags = toggleFlag(flags, configModes[i].flag);
        areFlagsInvalid = true;
      }
    }
  }
  else 
  {
    for (int i=0; i<2; i++)
      configModes[i].isChangingMode=false;
    Joystick.button(32,LOW);
  }

  if (areFlagsInvalid)
  {
    EEPROM.write(FLAGS_ADDRESS,flags);
  }

  // Wait
  delay(10);                                // Wait for 10ms
}

