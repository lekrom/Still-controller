// Options
#define USE_FLOW_SENSOR 1
#define FLOW_LITERS_PER_HOUR 0 // 0 for liters per minute

#define DEBUG         1   // 1 to enable debugging / sanity checks @ startup
#define DEBUG_UI      0   // 1 to enable debugging of UI thread
#define DEBUG_BUTTON  0   // 1 to enable debugging of UI thread
#define DEBUG_SAMPLE  0   // 1 to enable debugging of sample thread
#define DEBUG_ENCODER 0   // 1 to enable debugging of Encoder thread
#define DEBUG_CTRL    0   // 1 to enable debugging of CTRL thread
#define DEBUG_FLOW    1   // 1 to enable debugging of Flow sensor thread

// Hardware setup
#define TEMP_SENSOR_PIN 5
#define BUTTON_PIN      4
#define ENCODER_A_PIN   2
#define ENCODER_B_PIN   3
#define HEATER_PIN      7
#define FLOW_SENSOR_PIN 6
#define LCD_ADDRESS     0x27
#define LCD_COLS        16
#define LCD_ROWS        2

#define BOILER_SENSOR_INDEX 0
#define HEAD_SENSOR_INDEX   1
#define ENCODER_STEPS       1
#define INVERT_ENCODER      0

#define FLOW_SENSOR_SCALER  11

// Timing stuff (all params in ms)
#define DEBOUNCE_TIME           10  
#define UI_UPDATE_TIME          100 
#define PWM_CYCLE_TIME          2500
#define SAMPLE_PERIOD           300 
#define ENCODER_PERIOD          5
#define FLOW_SAMPLE_PERIOD      2000 
#define FLOW_DISPLAY_CYCLE_TIME 5000       

// defaults
#define DEFAULT_POWER_SETTING   0
#define DEFAULT_BOILER_SETTING  0
#define DEFAULT_HEAD_SETTING    0

#define TEMP_HYSTERESIS         0.5
#define TEMPERATURE_PRECISION   10

//limits
#define MAX_BOILER_TEMP         100
#define MAX_HEAD_TEMP           100



