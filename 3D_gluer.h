


//EasyDriver def
#define ED_STEP_PIN           3
#define ED_DIR_PIN            4
#define ED_MS1_PIN            5
#define ED_MS2_PIN            6
#define ED_ENABLE_PIN         8
#define ED_ENABLE_PIN_PORTB   0

#define ED_STEP_SIZE_FORWARD  20
#define ED_STEP_SIZE_BACKWARD 300

// Thermistor def
#define THERM_PIN         A0
#define THERM_NOM         100000
#define THERM_TEMP_NOM    25
#define THERM_NBR_SAMPLES 1
#define THERM_B_COEFF     4267
#define THERM_SERIES_RES  10000

// Potentiometer def
#define POT_TEMP          A1
#define POT_SPEED         A2


// Heater def
#define HEAT_ENABLE_PIN   7
#define HEAT_DEFAULT_VALUE  230
#define HEAT_HYSTERESIS   2
#define HEATER_WARM_ENOUGH_SCALE 0.8

// Pushbutton
#define PUSHBUTTON_STEP   9
#define PUSHBUTTON_STEP_PORTB   B00000010
#define PUSHBUTTON_ENABLE 2
#define LED_ENABLE        10
#define LED_ENABLE_PORTB  2

#define DEBUG             1



void setup_ed_stepper();
void reset_ed_pins();
void setup_ADC();
void setup_pushbutton();
void setup_heater();
float read_thermistor();
void heater_on();
void heater_off();
void ed_steps_forward();
void ed_steps_backward();
void setup_pushbutton();
int read_speed_pot();
int read_temp_pot();
