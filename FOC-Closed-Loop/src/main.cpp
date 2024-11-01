#include <Arduino.h>
#include <Wire.h>
#include "absolute_angle.h"

/* Reference:
 * https://electronoobs.com/eng_arduino_tut176.php */

// Some facts about the 3314 BLDC motor:
// It took 7 phase cycles to ratate the motor a full turn.
// The motor has 12 coils.


// Debug mode enables serial communication
// #define DBG

// Inputs and outputs
const int Enable_Pin = 8;	  // Enable pin for the driver
const int Motor_phase_A = 9;  // Pin for driver input of phase A
const int Motor_phase_B = 10; // Pin for driver input of phase B
const int Motor_phase_C = 11; // Pin for driver input of phase C

// PID parameters
const float Kp = 1;
const float Ki = 0.01;
const float Kd = 3;

// Variables used in the code

double phase_angle = 0;

uint16_t encoder_read = 0;
uint16_t encoder_read_r = 0;

absolute_angle field_abs_angle;	// The desired absolute angle of the rotor, executed by applying that direction of magnetic field
absolute_angle encoder_abs_read;	// The accumulated read value from the encoder
absolute_angle encoder_abs_angle;	// The absolute angle of the encoder, converted from encoder_abs_read
// absolute_angle encoder_abs_angle_r;
absolute_angle goal_abs_angle;

double error = 0;
double error_r = 0;

double field_offset = 0;	// The misalignment angle between encoder and coil, which can be calculated by align_encoder_and_coils()

uint32_t time_r = 0;
// double encoder_abs_angle_r = 0;

double error_sum = 0;

// Functions used in the code
void align_encoder_and_coils();
void apply_magnetic_field();
void get_angle();

void setup()
{
	#ifdef DBG
	Serial.begin(576000);
	#endif
	Wire.begin();

	// We need to set the PWM frequency to be the same for all 3 pins D9, D10 and D11
	TCCR0B = (TCCR0B & 0b11111000) | 0x03; // Changing would affect millis() and delay() so better to leave it default (0x03).
	TCCR1B = (TCCR1B & 0b11111000) | 0x01; // Set PWM frequency at 31250Hz for D9 and 10, (0x03 is default value, gives 490 Hz).
	TCCR2B = (TCCR2B & 0b11111000) | 0x01; // Set PWM frequency at 31250Hz for D11 D3, (0x03 is default value, gives 490 Hz).

	pinMode(Motor_phase_A, OUTPUT);
	pinMode(Motor_phase_B, OUTPUT);
	pinMode(Motor_phase_C, OUTPUT);
	pinMode(Enable_Pin, OUTPUT);
	digitalWrite(Enable_Pin, HIGH);

	align_encoder_and_coils();
}

void loop()
{
	goal_abs_angle.set_abs_angle(2000 + millis() / 1.0);
	get_angle();

	uint16_t time_elapsed = millis() - time_r;
	time_r += time_elapsed;
	error = constrain(encoder_abs_angle - goal_abs_angle, -2000, 2000);
	#ifdef DBG
	Serial.print(">error:");
	Serial.println(error);
	#endif
	// Kp term:
	double torque_angle = - error * Kp;	// The angle between the actual rotor angle and the applied field angle
	// Ki term:
	error_sum += error * time_elapsed;
	error_sum = constrain(error_sum, -500, 500);
	torque_angle -= error_sum * Ki;
	// Kd term:
	torque_angle -= Kd * (error - error_r) / time_elapsed;
	#ifdef DBG
	Serial.print(">Kd terms:");
	Serial.println(Kd * (error - error_r) / time_elapsed);
	#endif
	error_r = error;
	torque_angle = constrain(torque_angle, -12.85714, 12.85714);	// 360/7/4~=12.85714

	field_abs_angle = encoder_abs_angle + torque_angle;

	apply_magnetic_field(); // Function for moving the motor

}

void apply_magnetic_field()
{
	double field_abs_calibrated = field_abs_angle.fine - field_offset;

	phase_angle = field_abs_calibrated * 7.0;

	// Range calculation of Sine Signal
	phase_angle = phase_angle - ((long)phase_angle / 360) * 360; 	// Keep the value between 0 and 359

	// Calculate the PWM values for creating a sine signal (SPWM)
	int SINE_A_PWM = sin((double)(phase_angle) 		 * PI / 180) * 127.5 + 127.5; // Multiply by PI and divide by 180 in order to pass from degrees to radians
	int SINE_B_PWM = sin((double)(phase_angle + 120) * PI / 180) * 127.5 + 127.5; // Multiply by 127.5 and add 127.5 in order to keep the range between 0-255
	int SINE_C_PWM = sin((double)(phase_angle + 240) * PI / 180) * 127.5 + 127.5; // Sine values between -1 and 1 are placed between 0-255 for PWM.

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

	// Accumulate the encoder read value to get the absolute angle:
	int16_t delta = encoder_read - encoder_read_r;
	if(delta > 2047)
		-- encoder_abs_angle.coarse;
	else if(delta < -2047)
		++ encoder_abs_angle.coarse;
	encoder_abs_angle.fine = ((int32_t)encoder_read * 360) / 4096.0;
	encoder_read_r = encoder_read;
}

void align_encoder_and_coils(){
	#ifdef DBG
	Serial.println("Aligning encoder and coils, do not touch the motor...");
	#endif
	field_abs_angle.fine = 0;
	apply_magnetic_field();
	delay(500);
	uint32_t encoder_angle_avg = 0;
	for(int i = 0; i < 20; ++ i)
	{
		get_angle();
		encoder_angle_avg += encoder_read;
		delay(20);
	}
	encoder_angle_avg *= 360;
	encoder_angle_avg /= 4096.0;
	encoder_angle_avg /= 20.0;
	field_offset = encoder_angle_avg;
}