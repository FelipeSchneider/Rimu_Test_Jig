/*
 * timers.cpp
 *
 * Created: 08/03/2018 11:41:08
 *  Author: felip
 */ 
#include <Arduino.h>
#include "defines.h"
#include "variables.h"
#include "timers.h"
#include "motor_ctrl.h"

const PROGMEM uint16_t timerPeriod[721] = {1000, 1000, 56250, 37500, 28125, 22500, 18750, 16071, 14063, 12500, 11250, 10227, 9375, 8654, 8036, 7500, 56250, 52941, 50000, 47368, 45000, 42857,
	40909, 39130, 37500, 36000, 34615, 33333, 32143, 31034, 30000, 29032, 28125, 27273, 26471, 25714, 25000, 24324, 23684, 23077, 22500, 21951, 21429, 20930, 20455, 20000, 19565,
	19149, 18750, 18367, 18000, 17647, 17308, 16981, 16667, 16364, 16071, 15789, 15517, 15254, 15000, 14754, 14516, 14286, 14063, 13846, 13636, 13433, 13235, 13043, 12857, 12676,
	12500, 12329, 12162, 12000, 11842, 11688, 11538, 11392, 11250, 11111, 10976, 10843, 10714, 10588, 10465, 10345, 10227, 10112, 10000, 9890, 9783, 9677, 9574, 9474, 9375, 9278,
	9184, 9091, 9000, 8911, 8824, 8738, 8654, 8571, 8491, 8411, 8333, 8257, 8182, 8108, 8036, 7965, 7895, 7826, 7759, 7692, 7627, 7563, 7500, 7438, 7377, 7317, 7258, 7200, 7143,
	7087, 7031, 6977, 6923, 6870, 6818, 6767, 6716, 6667, 6618, 6569, 6522, 6475, 6429, 6383, 6338, 6294, 6250, 6207, 6164, 6122, 6081, 6040, 6000, 5960, 5921, 5882, 5844, 5806, 5769,
	5732, 5696, 5660, 5625, 5590, 5556, 5521, 5488, 5455, 5422, 5389, 5357, 5325, 5294, 5263, 5233, 5202, 5172, 5143, 5114, 5085, 5056, 5028, 5000, 4972, 4945, 4918, 4891, 4865, 4839,
	4813, 4787, 4762, 4737, 4712, 4688, 4663, 4639, 4615, 4592, 4569, 4545, 4523, 4500, 4478, 4455, 4433, 4412, 4390, 4369, 4348, 4327, 4306, 4286, 4265, 4245, 4225, 4206, 4186, 4167,
	4147, 4128, 4110, 4091, 4072, 4054, 4036, 4018, 4000, 3982, 3965, 3947, 3930, 3913, 3896, 3879, 3863, 3846, 3830, 3814, 3797, 3782, 3766, 3750, 3734, 3719, 3704, 3689, 3673, 3659,
	3644, 3629, 3614, 3600, 3586, 3571, 3557, 3543, 3529, 3516, 3502, 3488, 3475, 3462, 3448, 3435, 3422, 3409, 3396, 3383, 3371, 3358, 3346, 3333, 3321, 3309, 3297, 3285, 3273, 3261,
	3249, 3237, 3226, 3214, 3203, 3191, 3180, 3169, 3158, 3147, 3136, 3125, 3114, 3103, 3093, 3082, 3072, 3061, 3051, 3041, 3030, 3020, 3010, 3000, 2990, 2980, 2970, 2961, 2951, 2941, 2932,
	2922, 2913, 2903, 2894, 2885, 2875, 2866, 2857, 2848, 2839, 2830, 2821, 2813, 2804, 2795, 2786, 2778, 2769, 2761, 2752, 2744, 2736, 2727, 2719, 2711, 2703, 2695, 2687, 2679, 2671, 2663,
	2655, 2647, 2639, 2632, 2624, 2616, 2609, 2601, 2594, 2586, 2579, 2571, 2564, 2557, 2550, 2542, 2535, 2528, 2521, 2514, 2507, 2500, 2493, 2486, 2479, 2473, 2466, 2459, 2452, 2446, 2439,
	2432, 2426, 2419, 2413, 2406, 2400, 2394, 2387, 2381, 2375, 2368, 2362, 2356, 2350, 2344, 2338, 2332, 2326, 2320, 2314, 2308, 2302, 2296, 2290, 2284, 2278, 2273, 2267, 2261, 2256, 2250,
	2244, 2239, 2233, 2228, 2222, 2217, 2211, 2206, 2200, 2195, 2190, 2184, 2179, 2174, 2169, 2163, 2158, 2153, 2148, 2143, 2138, 2133, 2128, 2123, 2118, 2113, 2108, 2103, 2098, 2093, 2088,
	2083, 2079, 2074, 2069, 2064, 2059, 2055, 2050, 2045, 2041, 2036, 2032, 2027, 2022, 2018, 2013, 2009, 2004, 2000, 1996, 1991, 1987, 1982, 1978, 1974, 1969, 1965, 1961, 1957, 1952, 1948,
	1944, 1940, 1935, 1931, 1927, 1923, 1919, 1915, 1911, 1907, 1903, 1899, 1895, 1891, 1887, 1883, 1879, 1875, 1871, 1867, 1863, 1860, 1856, 1852, 1848, 1844, 1840, 1837, 1833, 1829, 1826,
	1822, 1818, 1815, 1811, 1807, 1804, 1800, 1796, 1793, 1789, 1786, 1782, 1779, 1775, 1772, 1768, 1765, 1761, 1758, 1754, 1751, 1748, 1744, 1741, 1737, 1734, 1731, 1727, 1724, 1721, 1718,
	1714, 1711, 1708, 1705, 1701, 1698, 1695, 1692, 1689, 1685, 1682, 1679, 1676, 1673, 1670, 1667, 1664, 1661, 1657, 1654, 1651, 1648, 1645, 1642, 1639, 1636, 1633, 1630, 1627, 1625, 1622,
	1619, 1616, 1613, 1610, 1607, 1604, 1601, 1599, 1596, 1593, 1590, 1587, 1585, 1582, 1579, 1576, 1573, 1571, 1568, 1565, 1563, 1560, 1557, 1554, 1552, 1549, 1546, 1544, 1541, 1538, 1536,
	1533, 1531, 1528, 1525, 1523, 1520, 1518, 1515, 1513, 1510, 1508, 1505, 1503, 1500, 1498, 1495, 1493, 1490, 1488, 1485, 1483, 1480, 1478, 1475, 1473, 1471, 1468, 1466, 1463, 1461, 1459,
	1456, 1454, 1452, 1449, 1447, 1445, 1442, 1440, 1438, 1435, 1433, 1431, 1429, 1426, 1424, 1422, 1420, 1417, 1415, 1413, 1411, 1408, 1406, 1404, 1402, 1400, 1398, 1395, 1393, 1391, 1389,
	1387, 1385, 1382, 1380, 1378, 1376, 1374, 1372, 1370, 1368, 1366, 1364, 1362, 1360, 1357, 1355, 1353, 1351, 1349, 1347, 1345, 1343, 1341, 1339, 1337, 1335, 1333, 1331, 1329, 1327, 1325,
	1324, 1322, 1320, 1318, 1316, 1314, 1312, 1310, 1308, 1306, 1304, 1302, 1301, 1299, 1297, 1295, 1293, 1291, 1289, 1288, 1286, 1284, 1282, 1280, 1278, 1277, 1275, 1273, 1271, 1269, 1268,
1266, 1264, 1262, 1261, 1259, 1257, 1255, 1253, 1252, 1250};

void initTimerMbase(void){
	TCCR1A = 0;
	TCCR1B = 0;
	gs_base_ctrl.speed = INITIAL_SPEED;
	updateMbaseSpeed(INITIAL_SPEED);
}

void updateMbaseSpeed(uint16_t speed){
	char old_SREG;
	speed = max(0,speed);
	speed = min(721,speed);
	#ifdef _DEBUG_COM_WHILE_SPEED_CTRL
		Serial.write(hi8(pgm_read_word_near(&timerPeriod[speed])));
		Serial.write(lo8(pgm_read_word_near(&timerPeriod[speed])));
	#endif // _DEBUG
	
	if(speed < INITIAL_SPEED){
		TCCR1B &= ~(1<<(CS10) | 1<<(CS11) | 1<<(CS12));
		//old_SREG = SREG;
		cli();											// Disable interrupts for 16 bit register access
		OCR1A = pgm_read_word_near(&timerPeriod[0]);
		sei();
		//SREG = old_SREG;
		TCCR1B |= 1<<(CS11);
	}
	else if (speed <= 15){
		TCCR1B &= ~(1<<(CS10) | 1<<(CS11) | 1<<(CS12));
		//old_SREG = SREG;
		cli();											// Disable interrupts for 16 bit register access
		OCR1A = pgm_read_word_near(&timerPeriod[speed]);
		sei();
		//SREG = old_SREG;
		TCCR1B |= 1<<(CS11);							// prescale by /8
	}
	else{
		TCCR1B &= ~(1<<(CS10) | 1<<(CS11) | 1<<(CS12));
		//old_SREG = SREG;
		cli();											// Disable interrupts for 16 bit register access
		OCR1A = pgm_read_word_near(&timerPeriod[speed]);
		sei();
		//SREG = old_SREG;
		TCCR1B |= 1<<(CS10);							// no prescale, full xtal
	}
}

void enableTimerMbase(void){
	TCNT1 = 0;
	TCCR1A |= 1<<(WGM10);
	TCCR1B |= 1<<(WGM13);
	TIMSK1 = 1<<(TOIE1);                                     // sets the timer overflow interrupt enable bit
}

void disableTimerMbase(void){
	TCNT1 = 0;
	TIMSK1 &= ~(1<<(TOIE1));
}



void initTimerMtop(void){
	TCCR3A = 0;
	TCCR3B = 0;
	gs_top_ctrl.speed = INITIAL_SPEED;
	updateMtopSpeed(INITIAL_SPEED);
}

void updateMtopSpeed(uint16_t speed){
	char old_SREG;
	speed = max(0,speed);
	speed = min(721,speed);
	
	if(speed < INITIAL_SPEED){
		TCCR3B &= ~(1<<(CS10) | 1<<(CS11) | 1<<(CS12));
		//old_SREG = SREG;
		cli();											// Disable interrupts for 16 bit register access
		OCR3A = pgm_read_word_near(&timerPeriod[0]);
		sei();
		//SREG = old_SREG;
		TCCR3B |= 1<<(CS11);							// prescale by /8
	}	
	else if (speed <= 15){
		TCCR3B &= ~(1<<(CS10) | 1<<(CS11) | 1<<(CS12));
		//old_SREG = SREG;
		cli();											// Disable interrupts for 16 bit register access
		OCR3A = pgm_read_word_near(&timerPeriod[speed]);
		sei();
		//SREG = old_SREG;
		TCCR3B |= 1<<(CS11);							// prescale by /8
	}
	else{
		TCCR3B &= ~(1<<(CS10) | 1<<(CS11) | 1<<(CS12));
		//old_SREG = SREG;
		cli();											// Disable interrupts for 16 bit register access
		OCR3A = pgm_read_word_near(&timerPeriod[speed]);
		sei();
		//SREG = old_SREG;
		TCCR3B |= 1<<(CS10);							// no prescale, full xtal
	}
}

void enableTimerMtop(void){
	TCNT3 = 0;
	TCCR3A |= 1<<(WGM30);
	TCCR3B |= 1<<(WGM33);
	//TCCR3A |= 1<<(WGM30);
	TIMSK3 = 1<<(TOIE3);                                     // sets the timer overflow interrupt enable bit
}

void disableTimerMtop(void){
	TCNT3 = 0;
	TIMSK3 &= ~(1<<(TOIE3));
}

void initTimerAcc(void){
	TCCR4A = 0;
	TCCR4B = 0;
	ICR4 = SPEED_UPDATE_TIMER_TOP;
	TCCR4B = 1<<(CS10);
}

void enableTimerAcc(void){
	TCNT4 = 0;
	TCCR4B |= 1<<(WGM43);
	TIMSK4 = 1<<(TOIE4);                                     // sets the timer overflow interrupt enable bit
}

void disableTimerAcc(void){
	TCNT4 = 0;
	TIMSK4 &= ~(1<<(TOIE4));
}



ISR(TIMER1_OVF_vect) {
	//A step is only given in the rising edge
//	CLEAR_NATIVE_LED;
	gs_base_ctrl.c_steps_made++;
	if (abs(gs_base_ctrl.prog_speed) >= INITIAL_SPEED){	// if the command is to really move the motor:
		limitBaseSteps();		//check if we can make one more step. Set the flags if we cannot, if we can, rise the step pin
		
	}
	if(gs_base_ctrl.c_steps_made >= gs_base_ctrl.prog_steps){
		disableTimerMbase();
		gs_base_ctrl.c_steps_made = 0;
		gs_base_ctrl.speed = 0;
		gs_base_ctrl.f_rdy_4_command = 1;		
	}
	delayMicroseconds(2);	
	CLEAR_MBASE_STEP_PIN;
}

ISR(TIMER3_OVF_vect) {
//	CLEAR_NATIVE_LED;
	gs_top_ctrl.c_steps_made++;
	if (abs(gs_top_ctrl.prog_speed) >= INITIAL_SPEED){
		SET_MTOP_STEP_PIN;
	}
	
	if(gs_top_ctrl.c_steps_made >= gs_top_ctrl.prog_steps){
		disableTimerMtop();
		gs_top_ctrl.c_steps_made = 0;
		gs_top_ctrl.speed = 0;
		gs_top_ctrl.f_rdy_4_command = 1;		
	}		
	delayMicroseconds(2);	
	CLEAR_MTOP_STEP_PIN; 
}

ISR(TIMER4_OVF_vect){
	if (abs(gs_base_ctrl.prog_speed) >= INITIAL_SPEED){
		if (gs_base_ctrl.c_steps_made <= gs_base_ctrl.n_to_cruse_speed){	//if we need to speed up
			gs_base_ctrl.speed = max(INITIAL_SPEED,gs_base_ctrl.speed);
			gs_base_ctrl.speed += ACC_SCALER;
			gs_base_ctrl.speed = min(721,gs_base_ctrl.speed);
			updateMbaseSpeed((uint16_t)(gs_base_ctrl.speed));		//apply the timer period table according to the actual speed
		}
		else if(gs_base_ctrl.c_steps_made >= gs_base_ctrl.n_to_start_breaking){	//if we net to reduce the angular speed
			gs_base_ctrl.speed = max(INITIAL_SPEED,gs_base_ctrl.speed);
			gs_base_ctrl.speed -= BREAKING_SCALER;
			gs_base_ctrl.speed = min(721,gs_base_ctrl.speed);
			updateMbaseSpeed((uint16_t)(gs_base_ctrl.speed));
		}
		else{
			updateMbaseSpeed(abs(gs_base_ctrl.prog_speed));
		}
	}
	else{
		updateMbaseSpeed(0);
	}
	
	if (abs(gs_top_ctrl.prog_speed) >= INITIAL_SPEED){
		if (gs_top_ctrl.c_steps_made <= gs_top_ctrl.n_to_cruse_speed){	//if we need to speed up
			SET_NATIVE_LED;
			gs_top_ctrl.speed = max(INITIAL_SPEED,gs_top_ctrl.speed);
			gs_top_ctrl.speed += ACC_SCALER;
			gs_top_ctrl.speed = min(721,gs_top_ctrl.speed);
			updateMtopSpeed((uint16_t)(gs_top_ctrl.speed));		//apply the timer period table according to the actual speed
		}
		else if(gs_top_ctrl.c_steps_made >= gs_top_ctrl.n_to_start_breaking){	//if we net to reduce the angular speed
			CLEAR_NATIVE_LED;
			gs_top_ctrl.speed = max(INITIAL_SPEED,gs_top_ctrl.speed);
			gs_top_ctrl.speed -= BREAKING_SCALER;
			gs_top_ctrl.speed = min(721,gs_top_ctrl.speed);
			updateMtopSpeed((uint16_t)(gs_top_ctrl.speed));
		}
		else{
			updateMtopSpeed(abs(gs_top_ctrl.prog_speed));
		}
	}
	else{
		updateMtopSpeed(0);
	}
	
	
}