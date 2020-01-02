// Includes

/*
 * TODO:
 * Implement timer for doing analog read for set temp and speed: DONE
 * Implement the speed control: IN PROGRESS
 * Implement the temp control: DONE
 */

#include "3D_gluer.h"


// Defines
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))


// Global variables
bool last_ed_on = false;
volatile bool enable = false;
volatile int8_t set_temp = 0;




void setup() {

#if DEBUG
  Serial.begin(115200);
  Serial.println("Setup start");
#endif

  setup_timer1();
  setup_enable_rutine();
  setup_ed_stepper();
  setup_heater();
  setup_ADC();
  setup_pushbutton_step();

  delay(500);

}



void loop() {

  if (enable == true) {
    float temp = read_thermistor();

    if  ( temp < (HEAT_DEFAULT_VALUE - HEAT_HYSTERESIS + set_temp) ) {
      heater_on();
    } else if ( temp > (HEAT_DEFAULT_VALUE + set_temp) ) {
      heater_off();
    }

#if 0
    Serial.print("Speed: ");
    Serial.println(set_speed);
    Serial.print("Temp: ");
    Serial.println(set_temp);
#endif

//    int pushbutton_pressed = digitalRead(PUSHBUTTON_STEP);
    int pushbutton_pressed = (PINB & PUSHBUTTON_STEP_PORTB) >> 1;
    if ( (pushbutton_pressed == LOW) && (temp > (HEAT_DEFAULT_VALUE + set_temp) * HEATER_WARM_ENOUGH_SCALE) ) {
      ed_steps_forward();
      last_ed_on = true;
    } else if (last_ed_on == true) {
      ed_steps_backward();
      last_ed_on = false;
    }
  } else {
#if DEBUG
    Serial.println("Not enabled");
    heater_off();
    delay(200);
#endif
  }

}




void setup_heater() {
#if DEBUG
  Serial.println("Setup heater");
#endif
  pinMode(HEAT_ENABLE_PIN, OUTPUT);
  heater_off();
}


void setup_ADC() {
#if DEBUG
  Serial.println("Setup thermistor");
#endif
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  ADCSRA |= bit (ADPS0) | bit (ADPS2);                  //  32
  analogReference(EXTERNAL);
}

float read_thermistor() {
//  uint8_t i;
//  float average;

//  for (i = 0; i < THERM_NBR_SAMPLES; i++) {
//    samples[i] = analogRead(THERM_PIN);
//    //delay(10);
//  }

  int reading = analogRead(THERM_PIN);

//  average = 0;
//  for ( i = 0; i < THERM_NBR_SAMPLES; i++) {
//    average += samples[i];
//  }
//  average /= THERM_NBR_SAMPLES;

//  // convert the value to resistance
//  average = 1023 / average - 1;
//  average = THERM_SERIES_RES / average;

  reading = 1023 / reading - 1;
  reading = THERM_SERIES_RES / reading;

  float steinhart;
  steinhart = reading / THERM_NOM;              // (R/Ro)
  steinhart = log(steinhart);                   // ln(R/Ro)
  steinhart /= THERM_B_COEFF;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (THERM_TEMP_NOM + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                  // Invert
  steinhart -= 273.15;                          // convert to C

#if DEBUG
  Serial.print("Temp: ");
  Serial.print(steinhart);
  Serial.println(" °C");
#endif

  return steinhart;
}


void heater_on() {
#if DEBUG
  //Serial.println("Heater on");
#endif
//  digitalWrite(HEAT_ENABLE_PIN, HIGH);
  SET(PORTD, HEAT_ENABLE_PIN);
}

void heater_off() {
#if DEBUG
  //Serial.println("Heater off");
#endif
//  digitalWrite(HEAT_ENABLE_PIN, LOW);
  CLR(PORTD, HEAT_ENABLE_PIN);
}

void setup_ed_stepper() {
#if DEBUG
  Serial.println("setup ED stepper");
#endif
  pinMode(ED_STEP_PIN, OUTPUT);
  pinMode(ED_DIR_PIN, OUTPUT);
  pinMode(ED_MS1_PIN, OUTPUT);
  pinMode(ED_MS2_PIN, OUTPUT);
  pinMode(ED_ENABLE_PIN, OUTPUT);

  reset_ed_pins();
}

void reset_ed_pins()
{
#if DEBUG
  Serial.println("Reset ED pins");
#endif
  digitalWrite(ED_STEP_PIN, LOW);
  digitalWrite(ED_DIR_PIN, LOW);
  digitalWrite(ED_MS1_PIN, LOW);
  digitalWrite(ED_MS2_PIN, LOW);
  digitalWrite(ED_ENABLE_PIN, HIGH);
}

void set_step_speed(uint8_t step_speed) {

   if(step_speed >= 75) {
      CLR(PORTD, ED_MS1_PIN);
      CLR(PORTD, ED_MS2_PIN);
  } else if (step_speed >= 50) {
      SET(PORTD, ED_MS1_PIN);
      CLR(PORTD, ED_MS2_PIN);
  } else if (step_speed >= 25) {
      CLR(PORTD, ED_MS1_PIN);
      SET(PORTD, ED_MS2_PIN);
  } else {
      SET(PORTD, ED_MS1_PIN);
      SET(PORTD, ED_MS2_PIN);
  }
}


void ed_steps_forward() {
//  digitalWrite(ED_ENABLE_PIN, LOW);
  CLR(PORTB,ED_ENABLE_PIN_PORTB);
  int x;
#if DEBUG
  Serial.println("Forward");
#endif
//  digitalWrite(ED_DIR_PIN, LOW); //Pull direction pin low to move "forward"
  CLR(PORTD, ED_DIR_PIN);
  for (x = 0; x < ED_STEP_SIZE_FORWARD; x++) //Loop the forward stepping enough times for motion to be visible
  {
//    digitalWrite(ED_STEP_PIN, HIGH); //Trigger one step forward
    SET(PORTD, ED_STEP_PIN);
    delay(1);
//    digitalWrite(ED_STEP_PIN, LOW); //Pull step pin low so it can be triggered again
    CLR(PORTD, ED_STEP_PIN);
    delay(1);
  }

//  digitalWrite(ED_ENABLE_PIN, HIGH);
    SET(PORTB, ED_ENABLE_PIN_PORTB);

}

void ed_steps_backward() {
//  digitalWrite(ED_ENABLE_PIN, LOW);
  CLR(PORTB, ED_ENABLE_PIN_PORTB);
  int x;
#if DEBUG
  Serial.println("Backward");
#endif
//  digitalWrite(ED_DIR_PIN, HIGH); //Pull direction pin high to move "backwards"
  SET(PORTD, ED_DIR_PIN);
  for (x = 0; x < ED_STEP_SIZE_BACKWARD; x++) //Loop the forward stepping enough times for motion to be visible
  {
//    digitalWrite(ED_STEP_PIN, HIGH); //Trigger one step forward
    SET(PORTD, ED_STEP_PIN);
    delay(1);
//    digitalWrite(ED_STEP_PIN, LOW); //Pull step pin low so it can be triggered again
    CLR(PORTD, ED_STEP_PIN);
    delay(1);
  }

//  digitalWrite(ED_ENABLE_PIN, HIGH);
  SET(PORTB, ED_ENABLE_PIN_PORTB);

}


void setup_pushbutton_step() {
#if DEBUG
  Serial.println("Pushbutton step setup");
#endif
  pinMode(PUSHBUTTON_STEP, INPUT_PULLUP);
}

void setup_enable_rutine() {
#if DEBUG
  Serial.println("Enable rutine setup");
#endif
  pinMode(LED_ENABLE, OUTPUT);
  pinMode(PUSHBUTTON_ENABLE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PUSHBUTTON_ENABLE), isr_enable_handler, RISING);
  enable = false;
}

void isr_enable_handler() {
  if (enable == true) {
    enable = false;
//    digitalWrite(LED_ENABLE, LOW);
    CLR(PORTB, LED_ENABLE_PORTB);

  } else {
    enable = true;
//    digitalWrite(LED_ENABLE, HIGH);
    SET(PORTB, LED_ENABLE_PORTB);
  }
}

/*
int read_speed_pot() {
  int value = analogRead(POT_SPEED);
  return map(value, 0, 1023, 0, 100);

}

int read_temp_pot() {
  int value = analogRead(POT_TEMP);
  return map(value, 0, 1023, -50, 50);
}
*/

void setup_timer1() {
  cli();

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();

}

ISR(TIMER1_COMPA_vect){
  int analog_speed = analogRead(POT_SPEED);
  int set_speed = map(analog_speed, 0, 1023, 0, 100);
  set_step_speed(set_speed);

  int analog_temp = analogRead(POT_TEMP);
  set_temp = map(analog_temp, 0, 1023, -50, 50);

}
