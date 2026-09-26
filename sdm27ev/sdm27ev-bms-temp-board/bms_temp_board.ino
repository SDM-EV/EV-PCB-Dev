const int mux1 = 13, mux2 = 10, // define mux input pins
          mux3 = 11, mux4 = 12; // we have 4 muxes

const int mux_s0 = 14, mux_s1 = 21, // define mux address pins
          mux_s2 = 47, mux_s3 = 48; // each mux has 16 (2^4) inputs

const int shutoff = 6, shutoff_led = 17, user_led = 18; // define shutoff and custom led outputs

const float max_temp = 60.0; // shutdown temperature in deg. C, should be 60 or less

const float init_temp = 25.0; // initial temperature to maintain accuracy in first few cycles

const float A = 1.3512 * pow(10,-3), B = 8.1227 * pow(10,-5), // coefficients for thermistor curve
            C = 2.0801 * pow(10,-5), D = -6.5251 * pow(10,-7); // Steinhart-Hart equation

const int samples = 5; // how many samples we want to store/average

int mux_inputs[4] = {mux1, mux2, mux3, mux4}; // mux pins

float temp_record[64][samples] = {{init_temp},{init_temp}}; // last recorded temp values

float temp_averages[64] = {init_temp};

float adc_to_temp (int adc_in) { // convert ADC reading (0-4095) to celsius
  float therm_R = (adc_in/4096.0) * 10000; // thermistor resistance, 10k is fixed resistor
  float ln_therm_R = log(therm_R); // calculate natural log once to save time
  // Steinhart-Hart equation to calculate temperature in celsius
  float therm_T = 1 / (A + B*ln_therm_R + C*pow(ln_therm_R,2) + D*pow(ln_therm_R,3)) + 273.15;
  return therm_T;
}

void shift_temps(float temp_record[64][samples]) { // shifts samples right to make space for new value
  for (int i = 0; i < 64; i++) {
    for (int j = samples; j > 0; j--) {
      temp_record[i][j] = temp_record[i][j-1];
    }
  }
}



void shutdown() { // write shutoff signal/led high and wait until reset
  digitalWrite(shutoff, HIGH);
  digitalWrite(shutoff_led, HIGH);
  exit(0);
}

void setup() {
  // set pinmodes
  pinMode(mux1, INPUT); pinMode(mux2, INPUT); pinMode(mux3, INPUT); pinMode(mux4, INPUT);
  pinMode(mux_s0, OUTPUT); pinMode(mux_s1, OUTPUT); pinMode(mux_s2, OUTPUT); pinMode(mux_s3, OUTPUT);
}

void loop() {
  // temperature data collection:
  for (int count = 0; count < 16; count++) { // count up binary address 0 to 15
    shift_temps(temp_record); // shift to make room for new data

    digitalWrite(mux_s0, count & 0b0001); // output binary address to each address pin
    digitalWrite(mux_s1, count & 0b0010); // & is bitwise AND
    digitalWrite(mux_s2, count & 0b0100);
    digitalWrite(mux_s3, count & 0b1000);

    for (int i = 0; i < 4; i++) { // convert to temperature and record samples
      temp_record[count*i][0] = adc_to_temp(analogRead(mux_inputs[i]));
    }

    // average samples
    for (int i = 0; i < 64; i++) {
      float temp_row_sum = 0; // initialize sum to 0
      for (int j = 0; j < samples; j++) {
        temp_row_sum += temp_record[i][j]; // add together all samples from one thermistor
      }
      temp_averages[i] = temp_row_sum / samples; // average samples and write to array
    }
  }

  // check for shutdown criteria on each temperature
  for (int i = 0; i < 64; i++) {
    if (temp_averages[i] >= max_temp) {
      shutdown();
    }
  }
}
