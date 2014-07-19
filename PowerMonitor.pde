#include <WProgram.h>
#include "Transfer.h"

#define OUTPUT_TRANSFER 1
#define OUTPUT_READABLE 2

#define OUTPUT_TYPE OUTPUT_TRANSFER

static const uint8_t vPins[] = {A4,A5,A6,A7};
static const uint8_t cPins[] = {A0,A1,A2,A3};

struct TransferData {
  float voltage[4];
  float current[4];
} data;

Transfer transfer;

/** This function is from http://hacking.majenko.co.uk/making-accurate-adc-readings-on-arduino.
 * It measures the actual value of VCC by comparing it with the Atmega328's internal 1.1V reference,
 * which is more accurate than the voltage regulator. This value can then be used to more
 * accurate measure the analog measurements. 
 *
 * The result is returned as a float.
 */
float readVcc() {
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Provides voltage in mV
  return result/1000.0f;
}

/** This function measures the voltage of the power source with the ADC. The
 * power source is connected through a voltage divider consisting of a 10Kohm
 * and 4.7Kohm resistor (2% tolerance). This produces a dividing factor:
 * 
 * V_out/V_in = R1 / (R1 + R2) = 4.7 / (4.7 + 10) = 0.3197
 * 
 * The Atmega328p has a 10-bit ADC and the analog reference voltage is Vcc. Therefore,
 * there are Vcc/1023 V/step. The volts (V) in this expression is the 
 * same as V_out. Therefore,
 * 
 * Vcc/1023 V_out/step x 1/0.3197 V_in/V_out = Vcc*0.00305761 V_in/step
 * 
 * This gives a max reading of 1023*Vcc*0.00306 = 15.6 V_in.
 * 
 * A low pass filter is used to smooth out the analog readings. It has a time constant
 * of 0.1 s and is implemented with a pretty standard discrete formula:
 * 
 * x(k+1) = x(k)*(1-alpha) + u(k+1)*alpha
 * 
 * where x is the output and u is the measured input. Alpha is calculated as
 * 
 * alpha = t/(t+T)
 * 
 * where T is the desired time constant (Tau) and t is period length of the 
 * sampling frequency. For the low pass with time constant of 0.25, and sampling
 * frequency of 100 Hz,
 * 
 * alpha = 0.01/(0.01+0.1) = 0.0909
 */
void measureVoltage(float &voltage,float dt,uint8_t anPin) {
  const static float k = 0.00305761;
  const static float tau = 0.25;
  
  static bool initialized = false;
  if ( !initialized ) {
    initialized = true;
    voltage = analogRead(anPin)*readVcc()*k;
  }
  
  float alpha = dt/(dt+tau);

  voltage = voltage*(1-alpha) + analogRead(anPin)*readVcc()*k*alpha;
}

/** This function measure the current supplied from the power source to the speed 
 * controller using a ACS715 current sensor chip. The output of this current 
 * sensor is 0.133 V/A. Like specified above, the Atmega328p has a 10 bit ADC and
 * 5.0 V reference voltage so that it has 0.004883 V/step. Combining this with the
 * current sensor relation provides:
 * 
 * Vcc/1023 V/step x 1/0.133 A/V = Vcc*0.0073498 A/step
 * 
 * The sensor measures 0-30A and is biased at 0.5 V.
 * 
 * A low pass filter is also used on the current measurement to smooth it out
 * slightly. The low pass filter has a time constant of 0.25 s.
 */
void measureCurrent(float &current,float dt,uint8_t anPin) {
  const static float k = 0.0073498;
  const static float tau = 0.25;
  const static int16_t center = 102; // equivalent to 0.5 V
  
  static bool initialized = false;
  if ( !initialized ) {
    initialized = true;
    current = (analogRead(anPin)-center)*readVcc()*k;
  }
  
  float alpha = dt/(dt+tau);
  
  current = current*(1-alpha) + (analogRead(anPin)-center)*readVcc()*k*alpha;
}

void setup() {
  Serial.begin(115200);

  transfer.setStream(&Serial);
}

void loop() {
  
  static long measurementTimer;
  static long outputTimer;
  
  float dt = float(micros()-measurementTimer)/1000000l;
  
  /** Measurement loop. I would put this in a timer interrupt but since these
   * are relatively long calculations the interrupt could block the RPM pulse
   * interrupt from occuring. */
  if ( dt > 0.005 ) {
    measurementTimer = micros();
    for (uint8_t i = 0 ; i < 4 ; i++) {
      measureVoltage(data.voltage[i],dt,vPins[i]);
      measureCurrent(data.current[i],dt,cPins[i]);
    }
  }
  
  /** Output loop. Output frequency can be adjusted. Currently, the output message
   * is 2+1+1+6*4+2 = 30 bytes = 240 bits/message. Therefore, at 115200 bps, 
   * 
   * 115200 bits/s x 1/240 messages/bit = 480 messages/s
   * 
   * can be sent under ideal conditions. In practice, this number will be lower. */
  if ( float(micros()-outputTimer)/1000000l > 0.1 ) {
    outputTimer = micros();
    
    switch ( OUTPUT_TYPE ) {
    case OUTPUT_TRANSFER:
			{
        transfer.send(&data);
			}
			break;
	  case OUTPUT_READABLE:
			{
        Serial.write(27);       // ESC command
        Serial.print("[2J");    // clear screen command
        Serial.write(27);
        Serial.print("[H");     // cursor to home command
        for (uint8_t i = 0 ; i < 4 ; i++) {
          Serial.print(i);Serial.print(": ");
          Serial.print(data.voltage[i]); Serial.print(" V\t");
				  Serial.print(data.current[i]); Serial.print(" A\t");
				  Serial.print(data.current[i]*data.voltage[i]); Serial.print(" W");
				  Serial.println("");
        }
			}
			break;
		default:
			Serial.println("Must define OUTPUT_TYPE.");    
    } 
  }
}
