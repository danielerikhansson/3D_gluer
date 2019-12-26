// Includes

#include "3D_gluer.h"


// Defines
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))


// Global variables
float temp = 0;
//int samples[THERM_NBR_SAMPLES];
bool last_ed_on = false;
bool enable = false;




void setup() {

#if DEBUG
  Serial.begin(115200);
  Serial.println("Setup start");
#endif

  setup_enable_rutine();
  setup_ed_stepper();
  setup_heater();
  setup_ADC();
  setup_pushbutton_step();

}





void loop() {

  if (enable == true) {
    temp = read_thermistor();
    int set_temp = read_temp_pot();

    if  (temp < HEAT_LOWER_LIMIT) {
      heater_on();
    } else if (temp > HEAT_UPPER_LIMIT) {
      heater_off();
    }

    int set_speed = read_speed_pot();

#if 0
    Serial.print("Speed: ");
    Serial.println(set_speed);
    Serial.print("Temp: ");
    Serial.println(set_temp);
#endif

//    int pushbutton_pressed = digitalRead(PUSHBUTTON_STEP);
    int pushbutton_pressed = (PINB & PUSHBUTTON_STEP_PORTB) >> 1;
    if ( (pushbutton_pressed == LOW) && (temp > (HEAT_UPPER_LIMIT) * HEATER_WARM_ENOUGH_SCALE) ) {
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
  ADCSRA |= bit (ADPS0) | bit (ADPS2);                 //  32
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
//  digitalWrite(ED_DIR_PIN, HIGH); //Pull direction pin high to move "forward"
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
  attachInterrupt(digitalPinToInterrupt(PUSHBUTTON_ENABLE), isr_handler, RISING);
  enable = false;
}

void isr_handler() {
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


int read_speed_pot() {
  int value = analogRead(POT_SPEED);
  return map(value, 0, 1023, 0, 100);

}

int read_temp_pot() {
  int value = analogRead(POT_TEMP);
  return map(value, 0, 1023, -50, 50);
}
