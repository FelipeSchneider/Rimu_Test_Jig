/*
 * uart.cpp
 *
 * Created: 28/02/2018 21:44:23
 *  Author: felip
 */ 

#include <Arduino.h>
#include "defines.h"
#include "variables.h"
#include "motor_ctrl.h"
#include "ctrl_giga.h"
#include "timers.h"
//#include "TimerOne.h"

//the motors have to stop after every command. to keep on motor steady for a period, send speed zero and in the number of steps the time (in microseconds) that the motor must stay steady
//the global state returns to idle if both motors are receive the order speed 0x8000 and steps equal to zero  

// Communication protocol
// First byte -> UART_HEADER_1
// Second byte -> UART_HEADER_COM for sending the squence of steps and UART_HEADER_START to start executing the squence of steps
// From the third byte, we start the action section: two bytes for the cruse speed of base (negative means one direction, positive another)
//    another two bytes for the number of steps in this direction (base). The next for bytes are the cruse speed of top and number of steps for top motor.
// So the protocol will be like this:
// HEADER1, HEADER2, S_BASE_HIGH, S_BASE_LOW, N_BASE_HIGH, N_BASE_LOW, S_TOP_HIGH, S_TOP_LOW, N_TOP_HIGH, N_TOP_LOW, ...N*8 Bytes ..., 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00
//							 int16_t				  uint16_t                 int16_t               uint16_t                      END_MSG - MATLAB SENDS FIRST THE LOW BYTE

//the acceleration and break is predefined by the program and its goal is to reduce vibration. Its module cannot be changed by communication


void treatSerialIncome(void){
	static uint16_t c_rec = 0;
	static uint16_t p_vector = 0;
	static uint8_t  c_end_msg = 0;
	uint8_t data = Serial.read();
	if ((gfs.state == IDLE)||(gfs.state == RECEIVING)){
		if(c_rec == 0){     //waiting for first header
			if (data == UART_HEADER_1){   c_rec = 1;  }   //if it is the correct first byte
			else{             Serial.println("Header errado");  }
		}                   //end of waiting for first header
		else if(c_rec == 1){          //waiting for second header
			if (data == UART_HEADER_COM){   //if it is the correct second byte
				c_rec = 2;
				gfs.state = RECEIVING;
			}
			else if(data == UART_HEADER_START){
				c_rec = 0;
				if(gfs.rdy_to_rotate == 1){		//first init of a command set, the next start will be given automatically in the main
					enableTimerMbase();
					enableTimerMtop();
					enableTimerAcc();
					SET_MOVING_LED_PIN;
					gfs.rdy_to_rotate = 0;
					gfs.state = ROTATING;
				}		
				else{                         
					Serial.println("Comandos nao enviados");
				}
			}
			else{
				c_rec = 0;
				p_vector = 0;
				c_end_msg = 0;
				gfs.state = IDLE;
				Serial.flush();
				Serial.println("Header errado");    
			}
		}                     //end of waiting for second header
		else{                 //dealing with the rest of the messages
			gv_ctrl_com[p_vector] = data;               //saving into the buffer
			if(((p_vector % 8) == 1)&&(data == END_MSG[1])){  //if the velocity starts with an 0x80, than we must end the command section
				c_end_msg = 2;									//TODO:: MELHORAR ESSA DETECÇÃO DE FIM DE MSG
			}
			else if(c_end_msg != 0){
				c_end_msg++;
				if (c_end_msg == 8){
					p_vector = 0;
					c_end_msg = 0;
					c_rec = 0;
					gfs.state = IDLE;
					gs_base_ctrl.f_rdy_4_command = 1;	//the commands will be evaluated at main loop
					gs_top_ctrl.f_rdy_4_command = 1;
					#ifdef _DEBUG
						Serial.println("fim da msg identificado");
					#endif // _DEBUG
					return;
				}
			}
			p_vector++;
			if(p_vector >= REC_VECTOR_SIZE) p_vector = REC_VECTOR_SIZE - 1;
		}           //end of dealing with the rest of the messages
	}             //if ((gfs.state == IDLE)||(gfs.state == RECEIVING)){
	else{           //if the system is not able to receive any data, flush the serial
		Serial.flush();
		#ifdef _DEBUG
			Serial.println("Msg desconsiderada, motor rodando");
		#endif // _DEBUG
	}
  
}

void readNewSetCommands(void){
	//get the commands for next speed and number of steps for the both motor
	gs_base_ctrl.prog_speed = (int16_t)(gv_ctrl_com[gc_ctrl_com] |((uint16_t)(gv_ctrl_com[gc_ctrl_com+1])<<8));
	gs_base_ctrl.prog_steps = (uint16_t)(gv_ctrl_com[gc_ctrl_com+2] |((uint16_t)(gv_ctrl_com[gc_ctrl_com+3])<<8));
	
	gs_top_ctrl.prog_speed = (int16_t)(gv_ctrl_com[gc_ctrl_com+4] |((uint16_t)(gv_ctrl_com[gc_ctrl_com+5])<<8));
	gs_top_ctrl.prog_steps = (uint16_t)(gv_ctrl_com[gc_ctrl_com+6] |((uint16_t)(gv_ctrl_com[gc_ctrl_com+7])<<8));
	
	gc_ctrl_com = gc_ctrl_com + 8;
}

uint8_t processCommands(s_motor_ctrl *s){
	//return 1 if we should keep turning
	float aux_speed = abs(s->prog_speed);
	
	if (s->prog_speed == 0x8000){	//the end of command set
		#ifdef _DEBUG
			Serial.println("Controle: fim dos comandos");
		#endif // _DEBUG
		return(0);
	}
	
	
	if (abs(s->prog_speed) < INITIAL_SPEED ){	//the speed zero, we should not move and deal with number of steps as milliseconds
		s->speed = 0;
		s->n_to_cruse_speed = 0;
		s->steps_breaking = 0;
	}
	else if(s->prog_speed >= INITIAL_SPEED){	//turning in the positive direction
		#ifdef _DEBUG
			Serial.println("Vel. Pos.");
		#endif // _DEBUG
		s->speed = INITIAL_SPEED;
		s->f_dir = DIR_POSITIVE;
		s->n_to_cruse_speed = (uint16_t)(((aux_speed*aux_speed) - INITIAL_SPEED_2)/DEN_CALC_ACC_STEPS); //TORRICELLI EQUATION
		s->steps_breaking = (uint16_t)(((aux_speed*aux_speed) - INITIAL_SPEED_2)/DEN_CALC_ACC_STEPS);
		if(s->prog_steps >= (s->steps_breaking+s->n_to_cruse_speed)){	//if we have enough time to speed up and break, we can go up to the programmed speed
			s->n_to_start_breaking = s->prog_steps - s->steps_breaking;
			#ifdef _DEBUG
				Serial.println("Velocidade de cruzeiro sem cortes");
			#endif // _DEBUG
		}
		else{			//if we do not have enough steps to speed up to programmed speed, then we must reduce it
			#ifdef _DEBUG
				Serial.println("Velocidade de cruzeiro com cortes");
			#endif // _DEBUG
			s->n_to_cruse_speed = RATIO_ACC_BREAKING * s->prog_steps;
			s->n_to_start_breaking = s->n_to_cruse_speed;
			s->steps_breaking = s->prog_steps - s->n_to_cruse_speed;
			s->prog_speed = sqrt(INITIAL_SPEED_2 + 2*ACC_MODULE*s->n_to_cruse_speed*D_THETA);				//TORRICELLI EQUATION
		}
		
	}
	else{												//NEGATIVE SPEED - be careful with signal
		#ifdef _DEBUG
			Serial.println("Vel. Neg.");
		#endif // _DEBUG
		s->speed = INITIAL_SPEED;
		s->f_dir = DIR_NEGATIVE;
		s->n_to_cruse_speed = (uint16_t)(((aux_speed*aux_speed) - INITIAL_SPEED_2)/DEN_CALC_ACC_STEPS); //TORRICELLI EQUATION
		s->steps_breaking = (uint16_t)(((aux_speed*aux_speed) - INITIAL_SPEED_2)/DEN_CALC_ACC_STEPS);
		if(s->prog_steps >= (s->steps_breaking+s->n_to_cruse_speed)){	//if we have enough time to speed up and break, we can go up to the programmed speed
			s->n_to_start_breaking = s->prog_steps - s->steps_breaking;
			#ifdef _DEBUG
				Serial.println("Velocidade de cruzeiro sem cortes");
			#endif // _DEBUG
		}
		else{			//if we do not have enough steps to speed up to programmed speed, then we must reduce it
			#ifdef _DEBUG
				Serial.println("Velocidade de cruzeiro com cortes");
			#endif // _DEBUG
			s->n_to_cruse_speed = RATIO_ACC_BREAKING*s->prog_steps;
			s->n_to_start_breaking = s->n_to_cruse_speed;
			s->steps_breaking = s->prog_steps - s->n_to_cruse_speed;
			s->prog_speed = sqrt(INITIAL_SPEED_2 + 2*ACC_MODULE*s->n_to_cruse_speed*D_THETA)*(-1);	
		}
	}
	#ifdef _DEBUG
		Serial.println("Numero de steps para velocidade de cruzeiro:");
		Serial.println(s->n_to_cruse_speed);
	#endif // _DEBUG
	
	return(1);
}