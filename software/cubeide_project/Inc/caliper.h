/*
 * caliper.h
 *
 *  Created on: Jul 27, 2019
 *      Author: Keshikan
 */

#ifndef CALIPER_H_
#define CALIPER_H_


//config
#define CFG_COMPENSATION_ON
//#define CFG_DEBUG_INDICATE_ON


#define SENSOR_NUM (2)

//12-bit ADC value
//MAIN...main scale
//SUB...sub scale

#define CALIB_MAXMIN_WAVE_NUM (10)

#define CALIB_DELTA_MAX_THR (15)
#define CALIB_DELTA_MIN_THR (10)
#define CALIB_DELTA_MEASURE_NUM (100)

#define CALIB_VALUE_DEFAULT_MAIN_MAX (2000)
#define CALIB_VALUE_DEFAULT_MAIN_MIN (1650)
#define CALIB_VALUE_DEFAULT_SUB_MAX (2000)
#define CALIB_VALUE_DEFAULT_SUB_MIN (1650)
#define CALIB_VALUE_DEFAULT_DELTA (0.0f)
#define CALIB_VALUE_DEFAULT_ALPHA (1.0f)
#define CALIB_VALUE_DEFAULT_BETA (0.0f)

#define SW_NUM (2) // a number of switches
#define HYSTERESIS (10)

#define LENGTH_PERIOD (6.015f)	//millimeter
#define LENGTH_QUARTER (LENGTH_PERIOD / 4.0f)

//for write and load
#define CALIB_DATA_ADDRESS (0x0801F800)
#define CALIB_DATA_CHECK_NUM (0x8765ABCD)

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef enum{
	STAT_INIT, STAT_A, STAT_B, STAT_C, STAT_D
}ScaleState;

typedef enum{
	SCALE_CALIB, SCALE_CALIB_FINISH, SCALE_WORK, SCALE_HOLD
}ScaleMode;

typedef enum{
	CALIB_INIT, CALIB_MAXMIN, CALIB_PHASE, CALIB_FINISHED, CALIB_NG
}CalibStatus;

typedef enum{
	MAXMIN_MAX, MAXMIN_MIN, MAXMIN_INIT
}CalibMaxminStatus;

typedef struct{

	ScaleState sstat, sstat_prev;
	ScaleMode smode;
	float length, accurate_length, zero_pos_length;
	bool iserr;

	//calib
	CalibStatus cstat;
	CalibMaxminStatus cmaxmin[SENSOR_NUM];
	uint32_t wave_cnt[SENSOR_NUM];


	//calibration value
	uint16_t max[SENSOR_NUM], min[SENSOR_NUM], middle[SENSOR_NUM];
	uint16_t amplitude[SENSOR_NUM];
	float delta;
	float alpha, beta;

}CaliperState;

typedef enum{
	SW_PUSH_EDGE,	//generated push event
	SW_UNPUSH_EDGE,	//generated unpush event
	SW_PUSH_CONTINUE,
	SW_UNPUSH_CONTINUE,
}SwitchState;

typedef struct{
	uint16_t max[SENSOR_NUM], min[SENSOR_NUM], middle[SENSOR_NUM];
	uint16_t amplitude[SENSOR_NUM];
	float delta;
	uint32_t chk_num;
}CalibrationData;



extern void cpInit();
extern void cpCalcLength(uint16_t main, uint16_t sub);
extern CalibStatus cpCalibration(uint16_t y[SENSOR_NUM]);
extern void cpCalibrationStart();

extern void cpApplyPhaseCompensation(uint16_t y[SENSOR_NUM]);

extern float cpGetLength();
extern float cpGetAccLength();
extern void cpSetZeroPosition();
extern ScaleMode cpGetCaliperMode();
extern void cpSetCaliperMode(ScaleMode smd);

extern CaliperState c_state;
extern CalibrationData calib_dat;

#endif /* CALIPER_H_ */
