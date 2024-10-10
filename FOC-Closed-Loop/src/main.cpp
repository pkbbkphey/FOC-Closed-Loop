#include <Arduino.h>
#include <Wire.h>

/* Reference:
 * https://electronoobs.com/eng_arduino_tut176.php */

// Some facts about the 3314 BLDC motor:
// It took 7 phase cycles to ratate the motor a full turn.
// The motor has 12 coils.


// Debug mode enables serial communication
#define DBG

// Inputs and outputs
const int potentiometer = A3; // The speed/angle potentiometer is connected on this pin
const int Enable_Pin = 8;	  // Enable pin for the driver
const int Motor_phase_A = 9;  // Pin for driver input of phase A
const int Motor_phase_B = 10; // Pin for driver input of phase B
const int Motor_phase_C = 11; // Pin for driver input of phase C

// Variables used in the code
int16_t SINE_A = 0;	   // Initial angle value of the phase A
int16_t SINE_B = 120;  // Initial angle value of the phase B
int16_t SINE_C = 240;  // Initial angle value of the phase C
int poles = 10;		   /*Amount of poles of the motor (change this value if the motor is not
					   getting to a full rotation. For example, my motor has 28 poles but I had
					   to add "11" in order to make a full rotation*/
uint32_t pot_read = 0; // Variable to store the pot ADC value.
uint16_t pot_angle;	   // Variable to store setpoint angle.

uint16_t encoder_read = 0;
uint16_t encoder_read_r = 0;

int64_t field_abs_angle = 0;	// The desired absolute angle of the rotor, executed by applying that direction of magnetic field
int64_t encoder_abs_read = 0;	// The accumulated read value from the encoder
int64_t encoder_abs_angle = 0;	// The absolute angle of the encoder, converted from encoder_abs_read

// Functions used in the code
void apply_magnetic_field();
void get_angle();

void setup()
{
	#ifdef DBG
	Serial.begin(115200);
	#endif
	Wire.begin();

	// We need to set the PWM frequency to be the same for all 3 pins D9, D10 and D11
	TCCR0B = (TCCR0B & 0b11111000) | 0x03; // Changing would affect millis() and delay() so better to leave it default (0x03).
	TCCR1B = (TCCR1B & 0b11111000) | 0x01; // Set PWM frequency at 31250Hz for D9 and 10, (0x03 is default value, gives 490 Hz).
	TCCR2B = (TCCR2B & 0b11111000) | 0x01; // Set PWM frequency at 31250Hz for D11 D3, (0x03 is default value, gives 490 Hz).

	pinMode(potentiometer, INPUT);
	pinMode(Motor_phase_A, OUTPUT);
	pinMode(Motor_phase_B, OUTPUT);
	pinMode(Motor_phase_C, OUTPUT);
	pinMode(Enable_Pin, OUTPUT);
	digitalWrite(Enable_Pin, HIGH);
}

void loop()
{
	get_angle();
	apply_magnetic_field(); // Function for moving the motor
	
	// // Read the potentiometer 10 times for better values
	// for (uint8_t i = 0; i < 10; i++)
	// {
	// 	pot_read += analogRead(potentiometer);
	// 	delayMicroseconds(100);
	// }
	// pot_read = pot_read / 10;
	// /*The ADC read with the arduino is from 0 to 1023. We need to pass from ADC read to angle of the motor
	//   For that we multiply by 360 whitch is a full sine rotation. But to make a full motor rotation, we need
	//   a few sine rotations, that's why we multiply by the "poles" variable */
	// pot_angle = (((float)pot_read) * (360 * poles) / 1024);
	// SINE_A = pot_angle; // Pass the degree information to the SINE_A variable

	field_abs_angle = 720 - millis() / 10;
}

void apply_magnetic_field()
{
	SINE_A = (field_abs_angle % 360) * 7;
	SINE_B = SINE_A + 120; // We have a 120 phase difference betweeen phase A and B
	SINE_C = SINE_B + 120; // We have a 120 phase difference betweeen phase B and C

	// Range calculation of Sine Signal
	SINE_A = SINE_A % 360; // Keep the values between 0 and 359
	SINE_B = SINE_B % 360; // Keep the values between 0 and 359
	SINE_C = SINE_C % 360; // Keep the values between 0 and 359

	// Calculate the PWM values for creating a sine signal (SPWM)
	int SINE_A_PWM = sin((double)SINE_A * PI / 180) * 127.5 + 127.5; // Multiply by PI and divide by 180 in order to pass from degrees to radians
	int SINE_B_PWM = sin((double)SINE_B * PI / 180) * 127.5 + 127.5; // Multiply by 127.5 and add 127.5 in order to keep the range between 0-255
	int SINE_C_PWM = sin((double)SINE_C * PI / 180) * 127.5 + 127.5; // Sine values between -1 and 1 are placed between 0-255 for PWM.

	analogWrite(Motor_phase_A, SINE_A_PWM * 0.7); // You might change the 0.7 value for more torque...
	analogWrite(Motor_phase_B, SINE_B_PWM * 0.7); // You might change the 0.7 value for more torque...
	analogWrite(Motor_phase_C, SINE_C_PWM * 0.7); // You might change the 0.7 value for more torque...
}

void get_angle()
{
	Wire.beginTransmission(0x36); // AS5600 adress
	Wire.write(0x0C);
	Wire.endTransmission();
	// requestFrom() returns the number of bytes returned from the peripheral device:
	Wire.requestFrom(0x36, 2, true);

	// The returned angle value of AS5600 is between 0 and 4095:
	encoder_read = (Wire.read() << 8) | Wire.read();

	#ifdef DBG
	Serial.print(">encoder_read:");	// Teleplot syntax
	Serial.println(encoder_read);
	#endif

	// Accumulate the encoder read value to get the absolute angle:
	int16_t delta = encoder_read - encoder_read_r;
	if(delta > 2047)
		encoder_abs_read += delta - 4096;
	else if(delta < -2047)
		encoder_abs_read += delta + 4096;
	else
		encoder_abs_read += delta;
	encoder_read_r = encoder_read;
	encoder_abs_angle = 360 * (encoder_abs_read / 4096) + (encoder_abs_read % 4096) * 360.0 / 4095.0;
	#ifdef DBG
	Serial.print(">encoder_abs_read:");
	Serial.println(long(encoder_abs_read));
	Serial.print(">encoder_abs_angle:");
	Serial.println(long(encoder_abs_angle));
	#endif
}