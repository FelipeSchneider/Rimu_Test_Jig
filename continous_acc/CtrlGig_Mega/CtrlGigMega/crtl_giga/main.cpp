/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

//http://www.instructables.com/id/How-to-Load-Programs-to-an-Arduino-UNO-From-Atmel-/
//or
//https://medium.com/jungletronics/how-to-load-programs-to-an-arduino-uno-from-atmel-studio-7-83c8dd8d175d
//http://nets-nuts.com.br/pt/eletronica/arduino-no-atmel-studio-7-parte-3/
//care must be taken with the actual serial baud rate
//to see how config the atmel studio for use with arduino

#include "defines.h"
#include "variables.h"
#include "motor_ctrl.h"
#include "uart.h"
#include "timers.h"

//TODO:: alterar o sistema de aceleração para uma aceleração constante baseada em um terceiro timer que só faria a aceleração e freio
//TODO:: alterar o TOP dos timers do registrador ICR para o OCnA


void intTimer(void);
void checkBaseBlock(void);

s_global_counters gcs;
s_global_flags gfs;
s_motor_ctrl gs_top_ctrl, gs_base_ctrl;

uint8_t gv_ctrl_com[REC_VECTOR_SIZE];
uint16_t gc_ctrl_com = 0;

void setup() {
	// put your setup code here, to run once:

	Serial.begin(SERIAL_BAUD);
	
	configPinMotors();
	pinMode(PIN_LED_NATIVE, OUTPUT);
	pinMode(PIN_LED_BASE_LOCK, OUTPUT);
	pinMode(PIN_MONING_LED, OUTPUT);
	
	digitalWrite(PIN_LED_NATIVE,LOW);
	digitalWrite(PIN_LED_BASE_LOCK,LOW);
	
	//using a modified version of timer one, for the improvement of program performance and reduce of time execution
	initTimerMbase();
	initTimerMtop();
	initTimerAcc();
	//enableTimerMbase();
	mTopSetMicroStep(SIXTEENTH_STEP);
	digitalWrite(MTOP_STEP,LOW);
	digitalWrite(MTOP_ENABLE,HIGH);
	//digitalWrite(MTOP_ENABLE,LOW);
	digitalWrite(MTOP_RESET,HIGH);
	digitalWrite(MTOP_DIR,DIR_POSITIVE);

	mBaseSetMicroStep(SIXTEENTH_STEP);
	digitalWrite(MBASE_STEP,LOW);
	digitalWrite(MBASE_ENABLE,HIGH);
	//digitalWrite(MBASE_ENABLE,LOW);
	digitalWrite(MBASE_RESET,HIGH);
	digitalWrite(MBASE_DIR,DIR_POSITIVE);
	
	gs_base_ctrl.f_dir = DIR_POSITIVE;
	gcs.base_limit = 0;
	gfs.state = IDLE;
}

void loop() {
	//check for serial income
	
	if (Serial.available()){
		treatSerialIncome();
	}
	
	
	
	//here we will do the decodification and all the calculations to start turning the motors
	//remembering the protocol:
	// HEADER1, HEADER2, S_BASE_HIGH, S_BASE_LOW, N_BASE_HIGH, N_BASE_LOW, S_TOP_HIGH, S_TOP_LOW, N_TOP_HIGH, N_TOP_LOW, ...N*8 Bytes ..., 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00
	//                int16_t         uint16_t           int16_t          uint16_t                      END_MSG	- MATLAB SENDS FIRST THE LOW BYTE
	if((gs_base_ctrl.f_rdy_4_command == 1)&&(gs_top_ctrl.f_rdy_4_command == 1)){
		uint8_t keep_turning_base, keep_turning_top;
		readNewSetCommands();					//get the commands for next speed and number of steps for the both motor
		gs_base_ctrl.f_rdy_4_command = 0;
		gs_top_ctrl.f_rdy_4_command = 0;
		
		#ifdef _DEBUG
			Serial.println("nova vel:");
			Serial.println(gs_base_ctrl.prog_speed);
	// 		Serial.write(lo8(gs_base_ctrl.prog_speed));
	// 		Serial.write(hi8(gs_base_ctrl.prog_speed));
		#endif // _DEBUG
		
		keep_turning_base = processCommands(&gs_base_ctrl);		//calculate how many steps we must do to reach the end of acceleration, how many to start deceleration, the cruse speed and the direction
		keep_turning_top = processCommands(&gs_top_ctrl);
		gfs.rdy_to_rotate = 1;
		
		digitalWrite(MBASE_ENABLE,LOW);			//enabling the drivers
		digitalWrite(MTOP_ENABLE,LOW);
		
		digitalWrite(MBASE_DIR,gs_base_ctrl.f_dir);
		digitalWrite(MTOP_DIR,gs_top_ctrl.f_dir);
		
		if((keep_turning_base == 1)&&(keep_turning_top == 1)&&(gfs.state == ROTATING)){	//The rotating status is given at treatSerialIncome(void) when an "St" is received
			#ifdef _DEBUG
				Serial.println("Continuar rodando");
			#endif // _DEBUG
			
			#ifdef ANSWER_COM
				Serial.write('N');
			#endif
			
			if (abs(gs_base_ctrl.prog_speed)>=INITIAL_SPEED){gs_base_ctrl.speed = INITIAL_SPEED;}		//TODO:: VERIFICAR A NECESSIDADE DISTO AQUI, JÁ DEVE TER SIDO FEITO EM PROCESSCOMMANDS
				else{										 gs_base_ctrl.speed = 0;}
			updateMbaseSpeed((abs(gs_base_ctrl.speed)));
			enableTimerMbase();
			
			if (abs(gs_top_ctrl.prog_speed)>=INITIAL_SPEED){gs_top_ctrl.speed = INITIAL_SPEED;}
			else{											gs_top_ctrl.speed = 0;}
			updateMtopSpeed((abs(gs_top_ctrl.speed)));
			enableTimerMtop();
			enableTimerAcc();
		}
		else{
			#ifdef _DEBUG
				if(gfs.state == ROTATING)	{
					Serial.println("terminando o controle:");
				}
				else{
					Serial.println("aguardando St");
				}
			#endif // _DEBUG
			if (gfs.state == ROTATING){
				#ifdef ANSWER_COM
					Serial.write('E');
				#endif
				gfs.state = IDLE;
				gc_ctrl_com = 0;
				gs_base_ctrl.c_steps_made = 0;
				gs_base_ctrl.speed = 0;
				cli();					//avoid going into the rx interruption when disabling the timer 1 here
				disableTimerMbase();
				disableTimerMtop();
				disableTimerAcc();
				sei();
				digitalWrite(MBASE_ENABLE,HIGH);			//disabling the drivers
				digitalWrite(MTOP_ENABLE,HIGH);
				CLEAR_MOVING_LED_PIN;
			}
			//SET_NATIVE_LED;
		}
	}
	
	checkBaseBlock();
}

//https://playground.arduino.cc/Code/Timer1

