/*
 * timers.h
 *
 * Created: 08/03/2018 11:40:54
 *  Author: felip
 */ 


#ifndef TIMERS_H_
#define TIMERS_H_

void initTimerMbase(void);
void updateMbaseSpeed(uint16_t speed);
void enableTimerMbase(void);
void disableTimerMbase(void);

void initTimerMtop(void);
void updateMtopSpeed(uint16_t speed);
void enableTimerMtop(void);
void disableTimerMtop(void);

void initTimerAcc(void);
void enableTimerAcc(void);
void disableTimerAcc(void);

#endif /* TIMERS_H_ */