//************************************************************************************************
// Includes
#include "config.h"
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Bounce2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Encoder.h>

#if USE_FLOW_SENSOR == 1
  #include <PinChangeInterrupt.h>
#endif

#define byte 	uint8_t

//************************************************************************************************
// Global Variables

// Thread timers
unsigned long UI_update_timer = 0;
unsigned long Sample_timer = 0;
unsigned long Encoder_timer = 0;

#if USE_FLOW_SENSOR == 1
  unsigned long Flow_timer = 0;
  unsigned long flow_display_timer = 0;

  // Flow sensor stuff
  volatile unsigned int Flow_counter = 0;
  float Flow_rate = 0;        // l / min
#endif

// State machine stuff - control thread
#define STATE_OFF         0
#define STATE_CONST_POWER 1
#define STATE_BOIL_CTRL   2
#define STATE_HEAD_CTRL   3
byte control_state = STATE_OFF;

// Error codes
#define ERROR_OK            0
#define ERROR_BOIL_OVERTEMP 1
#define ERROR_HEAD_OVERTEMP 2
#define ERROR_SENSOR        3
byte error_code = ERROR_OK;

// Control variables
float  boiler_temp;
float  head_temp;
int8_t  power_setting = DEFAULT_POWER_SETTING;
float  boiler_setting = DEFAULT_BOILER_SETTING;
float  head_setting = DEFAULT_HEAD_SETTING;
unsigned long PWM_timer = 0;

// Flags
bool boiler_sensor_connected = false;
bool head_sensor_connected = false;
bool heater_on = 0;
bool prev_button = 0;
bool button = 0;
bool button_event = 0;
bool sensor_error = 0;
bool head_temp_error = 0;
bool boiler_temp_error = 0;

#if USE_FLOW_SENSOR == 1
  bool flow_dislay_flag = 0;
#endif

// Encoder stuff
int EncoderPosition =0;
int OldEncoderPosition = 0;
int8_t Encoder_Increment = 0;

// LCD Text - used to check if text has changed and needs updating
String Line1_Prev = "";
String Line2_Prev = "";

//************************************************************************************************
// Objects
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
Bounce debouncer = Bounce(); 
Encoder EnCoder(ENCODER_A_PIN, ENCODER_B_PIN);
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

//************************************************************************************************
// Functions
//************************************************************************************************//
// Function to scan the I2C bus, sanity check for LCD at startup
byte scan_I2C_bus(void)
{
  byte I2C_adress,error,nDevices=0;
  
  Wire.begin();

    for (I2C_adress = 1; I2C_adress < 127;I2C_adress++)
    {
      Wire.beginTransmission(I2C_adress);
      error = Wire.endTransmission();

      if (error == 0)
      {
        #if DEBUG == 1
          Serial.print("I2C device found at address 0x");
          if (I2C_adress<16) Serial.print("0");
            Serial.print(I2C_adress,HEX);
        #endif
        nDevices++;
      }

      #if DEBUG == 1
      else if (error==4)
      {
        Serial.print("Unknown error at address 0x");
        if (I2C_adress<16) Serial.print("0");
        Serial.println(I2C_adress,HEX);
      }                 
      #endif                
    }
    
    #if DEBUG == 1
      if (nDevices == 0) Serial.println("No I2C devices found\n");
      else Serial.println("I2C scan completed\n");                  
    #endif        
      
return nDevices;
}

//************************************************************************************************//
// Helper function - Convert OneWire adress (temp sensors) to a user friendly hex version
String OneWireAddress(DeviceAddress deviceAddress)
{

String Result = "";

  for (byte i = 0; i < 8; i++)
  {    
    Result += String(deviceAddress[i],HEX);
  }
  Result.toUpperCase();
  return Result; 
}

//************************************************************************************************//
// Scan the OneWire bus - temp sensor sanity check at startup
byte scan_temp_sensors(void)
{
  DeviceAddress tempDeviceAddress;
  byte numberOfDevices = sensors.getDS18Count();
  byte temp_resolution;
  float tempC=0;
  
  // locate devices on the bus
    #if DEBUG == 1
      Serial.println("Locating onewire devices...");  
      Serial.print("Found ");
      Serial.print(numberOfDevices, DEC);
      Serial.println(" devices.");
    #endif
 
    // Loop through each device, print out address
    for(byte i=0;i<numberOfDevices; i++)
    {
      // Search the wire for address
      if(sensors.getAddress(tempDeviceAddress, i))
	      {
          #if DEBUG == 1       
		        Serial.print("Found device (");
		        Serial.print(i, DEC);
		        Serial.print(") with address: ");
            Serial.print(OneWireAddress(tempDeviceAddress));
		        Serial.println();
		      #endif

          temp_resolution = sensors.getResolution(tempDeviceAddress);

          if (temp_resolution != TEMPERATURE_PRECISION)
          {
            sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
              #if DEBUG == 1                          
                Serial.print("Updating resolution from ");
                Serial.print(temp_resolution,DEC);
                Serial.print("bits to ");
                Serial.print(TEMPERATURE_PRECISION,DEC);
                Serial.println(" bits");
              #endif                           
          }

            tempC = sensors.getTempC(tempDeviceAddress);
            if(tempC == DEVICE_DISCONNECTED_C) 
              {
                #if DEBUG == 1             
                  Serial.println("Error: Could not read temperature data");
                #endif
              }
            else
            {
              #if DEBUG == 1
                Serial.print("Current Temperature: ");
                Serial.print(tempC);
                Serial.println(" C \n");
              #endif
            }         
	      }
      else
      {
        #if DEBUG == 1      
		      Serial.print("Found ghost device at ");
		      Serial.print(i, DEC);
		      Serial.println(" but could not detect address. Check power and cabling");
        #endif
	    }
    }
    
  return numberOfDevices;
}

//************************************************************************************************
// Sample thread function - executes periodically to read the temp sensors
// Sample rate determined by SAMPLE_PERIOD in config.h
void Sample_thread(void)
{
  DeviceAddress tempDeviceAddress;

  #if DEBUG_SAMPLE == 1 
    unsigned long start_TS = millis();
  #endif
  
    sensors.requestTemperatures(); // Send the command to get temperatures
    
   if(sensors.getAddress(tempDeviceAddress, BOILER_SENSOR_INDEX))
   {
      boiler_temp = sensors.getTempCByIndex(BOILER_SENSOR_INDEX);         
      boiler_sensor_connected = true;
   }
   else boiler_sensor_connected = false;

   if(sensors.getAddress(tempDeviceAddress, HEAD_SENSOR_INDEX))
   {
      head_temp = sensors.getTempCByIndex(HEAD_SENSOR_INDEX);         
      head_sensor_connected = true;
   }
  else head_sensor_connected = false;

    sensor_error = head_sensor_connected || boiler_sensor_connected;


   #if DEBUG == 1
    #if DEBUG_SAMPLE == 1
      unsigned long End_TS = millis();
      Serial.print("Sample Thread took ");
      Serial.print(End_TS - start_TS);
      Serial.print("ms ");
      Serial.print("BT: ");
      Serial.print(boiler_temp);
      Serial.print("C ");
      Serial.print("HT: ");
      Serial.print(head_temp);
      Serial.println("C");
    #endif
  #endif
}

//************************************************************************************************
//Interrupt service routine for the flow sensor, increment pulse counter, this value is used
// in the Flow sensor thread to calculate the flow rate
#if USE_FLOW_SENSOR == 1
void Flow_sensor_ISR(void)
{
    cli();

      Flow_counter++;

    sei();
}
#endif

//************************************************************************************************
// Flow sensor thread function - executes periodically to calculate the flow rate
// Sample rate determined by FLOW_SAMPLE_PERIOD in config.h
#if USE_FLOW_SENSOR == 1
void Flow_sensor_thread(void)
{
  unsigned int pulses = Flow_counter;
  Flow_counter = 0;

   #if DEBUG == 1
    #if DEBUG_FLOW == 1
      unsigned long start_TS = millis();
    #endif
  #endif

    float Flow_frequency = ((float)pulses / FLOW_SAMPLE_PERIOD)*1000;  
    Flow_rate = Flow_frequency / FLOW_SENSOR_SCALER;  

   #if DEBUG == 1
    #if DEBUG_FLOW == 1
      unsigned long End_TS = millis();
      Serial.print("Flow Thread took ");
      Serial.print(End_TS - start_TS);
      Serial.println("ms");
      Serial.print("Flow: ");
      Serial.print(Flow_rate);
      Serial.print(" l/min, pulses: ");
      Serial.print(pulses);
      Serial.print(" Frequency: ");
      Serial.print(Flow_frequency);
      Serial.print(" Sample period: ");
      Serial.println(FLOW_SAMPLE_PERIOD);
    #endif
   #endif
}
#endif

//************************************************************************************************
// Control thread function - Temperature / PWM control of heating element 
//  as well as error checking is done here
//  executes on each program cycle

void CTRL_thread(void) 
{
  float active_temp = 0;
  float active_temp_setting = 0;
  unsigned long PWM_On_time;
  unsigned long PWM_Off_time;
  bool temp_ctrl_flag;

    switch (control_state)
    {
      case STATE_OFF:
            PWM_timer = 0;
            heater_on = 0;
            temp_ctrl_flag = 0;
      break;
      case STATE_CONST_POWER:   
          temp_ctrl_flag = 0;       
          PWM_On_time = (power_setting * PWM_CYCLE_TIME) / 100;
          PWM_Off_time = PWM_CYCLE_TIME - PWM_On_time;
    
            if (heater_on)
            {
              if (millis() - PWM_On_time > PWM_timer)
              {
                heater_on = 0;
                PWM_timer = millis();

                #if DEBUG == 1
                  #if DEBUG_CTRL == 1
                    Serial.print("CONST POWER: ");
                    Serial.print("Heater OFF @ ");
                    Serial.print(millis());
                    Serial.println("ms");
                    Serial.print(PWM_On_time);
                    Serial.print("ms, Off time: ");
                    Serial.print(PWM_Off_time);
                    Serial.println("ms");
                  #endif
                #endif
              }
            }
            else 
            {
              if (millis() - PWM_Off_time > PWM_timer)
              {
                heater_on = 1;
                PWM_timer = millis();

                #if DEBUG == 1
                  #if DEBUG_CTRL == 1
                    Serial.print("CONST POWER: ");
                    Serial.print("Heater ON @ ");
                    Serial.print(millis());
                    Serial.print("ms ");
                    Serial.print("On time: ");
                    Serial.print(PWM_On_time);
                    Serial.print("ms, Off time: ");
                    Serial.print(PWM_Off_time);
                    Serial.println("ms");
                  #endif
                #endif                
              }
            }
            
      break;
      case STATE_BOIL_CTRL:
            active_temp = boiler_temp;
            active_temp_setting = boiler_setting;
            temp_ctrl_flag = 1;
      break;
      case STATE_HEAD_CTRL:
            active_temp = head_temp;
            active_temp_setting = head_setting;
            temp_ctrl_flag = 1;
      break;     
    }

    if (temp_ctrl_flag)
    {
      if (heater_on)
      { 
        if (active_temp >= active_temp_setting + TEMP_HYSTERESIS)
        {
          heater_on = 0;
          #if DEBUG == 1
            #if DEBUG_CTRL == 1
              if (control_state == STATE_HEAD_CTRL) Serial.print("Head Control - ");
              else Serial.print("Boil Control - ");
              
              Serial.print("Heater off @" );
              Serial.print(active_temp);
              Serial.println("C");
            #endif
          #endif          
        }
      }
      else
      {
       if (active_temp <= active_temp_setting - TEMP_HYSTERESIS)
        {
          heater_on = 1;
          #if DEBUG == 1
            #if DEBUG_CTRL == 1
              if (control_state == STATE_HEAD_CTRL) Serial.print("Head Control - ");
              else Serial.print("Boil Control - ");
              
              Serial.print("Heater on @" );
              Serial.print(active_temp);
              Serial.println("C");
            #endif
          #endif             
        }     
      }
    }

  // Handle user inputs
   if (button_event)
   {
     control_state++;
     if (control_state > STATE_HEAD_CTRL) control_state = STATE_OFF;
     button_event = 0;

      #if DEBUG == 1
        #if DEBUG_CTRL == 1
          Serial.print("control state: ");
          Serial.println(control_state);
        #endif
      #endif
   }

   if (Encoder_Increment != 0)
    {
      switch (control_state)
      {
        case STATE_CONST_POWER:
              power_setting += Encoder_Increment;
              if (power_setting > 100) power_setting = 100;
              else if (power_setting < 0) power_setting = 0;
        break;
        case STATE_BOIL_CTRL:
              boiler_setting += Encoder_Increment;
              if (boiler_setting > 100) boiler_setting = 100;
              else if (boiler_setting < 0) boiler_setting = 0;
        break;
        case STATE_HEAD_CTRL:
              head_setting += Encoder_Increment;
              if (head_setting > 100) head_setting = 100;
              else if (head_setting < 0) head_setting = 0;
        break;        
      }
      Encoder_Increment = 0; 
    } 

// Error checking
  sensor_error  = !(head_sensor_connected && boiler_sensor_connected);

  if (head_temp > MAX_HEAD_TEMP) head_temp_error = true;
  else head_temp_error = false;

  if (boiler_temp > MAX_BOILER_TEMP) boiler_temp_error = true;
  else boiler_temp_error = false;

  if (sensor_error) error_code = ERROR_SENSOR;
  else if (head_temp_error) error_code = ERROR_HEAD_OVERTEMP;
  else if (boiler_temp_error) error_code = ERROR_BOIL_OVERTEMP;
  else error_code = ERROR_OK;

  if (error_code != ERROR_OK)
  {
    control_state = STATE_OFF;
  } 

    digitalWrite(HEATER_PIN,heater_on);
    digitalWrite(LED_BUILTIN,heater_on);
}

//************************************************************************************************
// Read and debounce the encoder button to generate button events, use bounce2 library
void Read_button(void)
{
   debouncer.update();

    if (debouncer.read() == 1)
    {
       button = 0; 
    }
    else
    {
      button = 1;
    }
    
    if (button != prev_button)
    {      
        if (button)
        {
        #if DEBUG == 1
          #if DEBUG_BUTTON == 1
            Serial.println("Button pressed");     
          #endif   
         #endif                
        }
        else 
        {
          #if DEBUG == 1
            #if DEBUG_BUTTON == 1
              Serial.println("Button released");
            #endif
          #endif        
          #if USE_FLOW_SENSOR == 1
            if (flow_dislay_flag)
            {
              // revert to temp display when button input is received
              flow_dislay_flag = 0;
              flow_display_timer = millis();
            }
            else
            {
              button_event = 1;
            }
          #else
          button_event = 1;
          #endif
        }      
    }
    prev_button = button;  
}

//************************************************************************************************
// Rotary encoder thread function - Handles user input via the rotary encoder
//  Sample rate determined by ENCODER_PERIOD in config.h
void Encoder_thread(void)
{

  EncoderPosition = EnCoder.read(); 

  if (EncoderPosition != OldEncoderPosition) 
  {

    #if INVERT_ENCODER == 1
      if (EncoderPosition > OldEncoderPosition) Encoder_Increment = -1;
      else Encoder_Increment = 1;
    #else
      if (EncoderPosition > OldEncoderPosition) Encoder_Increment = 1;
      else Encoder_Increment = -1;    
    #endif
    
    OldEncoderPosition = EncoderPosition;   

    #if USE_FLOW_SENSOR == 1
      if (flow_dislay_flag)
      {
        // revert to temp display when encoder input is received
        flow_dislay_flag = 0;
        flow_display_timer = millis();
      }
    #endif

    #if DEBUG == 1
      #if DEBUG_ENCODER == 1
        Serial.print("Enc Position: ");
        Serial.print(EncoderPosition);
        Serial.print(" Enc Inc: ");
        Serial.println(Encoder_Increment);  
      #endif    
    #endif
  }  
}

//************************************************************************************************
// LCD Display thread function - Handles LCD text and update the display if text changes 
//  Refresh rate determined by UI_UPDATE_TIME in config.h
void Display_thread(void)
{
  String  line1 = "";
  String  line2 = "";

  #if DEBUG == 1
    #if DEBUG_UI == 1
      unsigned long start_TS = millis();
    #endif
  #endif

    switch (error_code)
    {
      case ERROR_SENSOR:
        line1 = "  SENSOR ERROR  ";

        if (!boiler_sensor_connected && head_sensor_connected) 
          line2 = " Boiler sensor  ";
        else if (!head_sensor_connected && boiler_sensor_connected) 
          line2 = " Head sensor    ";
        else                             
          line2 = " Both sensors   ";
      break;
      case ERROR_BOIL_OVERTEMP:
        line1 = " BOILER OVERTEMP";
        line2 = "     ERROR      ";
      break;
      case ERROR_HEAD_OVERTEMP:
        line1 = "  HEAD OVERTEMP ";
        line2 = "     ERROR      ";
      break;
      case ERROR_OK:
          switch (control_state)
            {
              case STATE_OFF:
              line1 = "   ---OFF---    ";
              break;
              case STATE_CONST_POWER:
              line1 = "Power CTRL: " + String(power_setting) +"%  ";
              break;
              case STATE_BOIL_CTRL:
              line1 = "Boiler CTRL:" + String(boiler_setting) +"C  ";
              break;
              case STATE_HEAD_CTRL:
              line1 = "Head CTRL:" + String(head_setting) + "C    ";
              break;
            } 

        #if USE_FLOW_SENSOR == 1
          if (flow_dislay_flag)
          {
            #if FLOW_LITERS_PER_HOUR == 1
              line2 = "Flow: " + String(Flow_rate * 60) + " l/h";
            #else
              line2 = "Flow: " + String(Flow_rate) + " l/m  ";
            #endif
          }
          else
          {
            line2 = "B:" + String(boiler_temp) + " H:" + String(head_temp);            
          } 
        #else
            line2 = "B:" + String(boiler_temp) + " H:" + String(head_temp);  
        #endif
      break;
    }
    
    if (Line1_Prev != line1)
    {
      lcd.setCursor(0,0);
      lcd.print(line1);
    }
    Line1_Prev = line1;

    if (Line2_Prev != line2)
    {
      lcd.setCursor(0,1);
      lcd.print(line2);
    }
    Line2_Prev = line2;

  #if DEBUG == 1
    #if DEBUG_UI == 1
      unsigned long End_TS = millis();
      Serial.print("UI Thread took ");
      Serial.print(End_TS - start_TS);
      Serial.println("ms");
    #endif
  #endif

}

//************************************************************************************************
void setup(void)
{

  #if DEBUG == 1
    // Serial comms used for debugging purposes only
    Serial.begin(115200);
  #endif

  // Configure inputs
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(ENCODER_A_PIN,INPUT_PULLUP);
  pinMode(ENCODER_B_PIN,INPUT_PULLUP);

  pinMode(FLOW_SENSOR_PIN,INPUT_PULLUP);

  // Configure outputs - use built in led to indicate heating 
  pinMode(HEATER_PIN,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);  
  digitalWrite(HEATER_PIN,0);

  // Configure debouncing object
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(DEBOUNCE_TIME); // interval in ms

  // Start the Temp sensors
  sensors.begin();

  // Do I2C and OneWire sanity checks
  scan_I2C_bus();
  scan_temp_sensors();

  // Start the LCD and display welcome screen
  lcd.begin();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Morri Still CTRL");
  lcd.setCursor(0,1);
  lcd.print(" ...Welcome!... ");
  delay(1000);

  // Setup flow sensor - interrupt driven, /use Pin change int library
  #if USE_FLOW_SENSOR == 1
    attachPCINT(digitalPinToPCINT(FLOW_SENSOR_PIN), Flow_sensor_ISR, RISING);
    sei();
  #endif

}

//************************************************************************************************
void loop(void)
{
  Read_button();

  if (millis() - Encoder_timer > ENCODER_PERIOD)
  {
    Encoder_thread();
    Encoder_timer = millis();
  }

  if (millis() - Sample_timer > SAMPLE_PERIOD)
  {
    Sample_thread();
    Sample_timer = millis();
  }

  CTRL_thread();

  #if USE_FLOW_SENSOR == 1
  if (millis() - Flow_timer > FLOW_SAMPLE_PERIOD)
  {
    Flow_sensor_thread();
    Flow_timer  = millis();
  }

  if (millis() - flow_display_timer > FLOW_DISPLAY_CYCLE_TIME)
  {
    flow_dislay_flag = !flow_dislay_flag;
    flow_display_timer = millis();
  } 
  #endif

  if (millis() - UI_update_timer > UI_UPDATE_TIME)
  {
    Display_thread();
    UI_update_timer = millis();
  }
}
