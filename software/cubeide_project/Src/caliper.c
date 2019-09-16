/*
 * caliper.c
 *
 *  Created on: Jul 27, 2019
 *      Author: Keshikan
 */

#include "caliper.h"

CaliperState c_state;
CalibrationData calib_dat;

uint32_t maxmin_wave_cnt[SENSOR_NUM];
uint32_t delta_cnt;
float delta_sum;

void cpInit()
{
	c_state.sstat = STAT_INIT;
	c_state.sstat_prev = STAT_INIT;
	c_state.smode = SCALE_WORK;
	c_state.length = c_state.accurate_length = c_state.zero_pos_length = 0.0f;
	c_state.iserr = false;

	//calib
	c_state.cstat = CALIB_INIT;
	for(uint32_t i=0; i<SENSOR_NUM; i++){
		c_state.cmaxmin[i] = MAXMIN_INIT;
		c_state.wave_cnt[i] = 0;
	}

	//calib data set
	c_state.max[0] = CALIB_VALUE_DEFAULT_MAIN_MAX;
	c_state.min[0] = CALIB_VALUE_DEFAULT_MAIN_MIN;
	c_state.max[1] = CALIB_VALUE_DEFAULT_SUB_MAX;
	c_state.min[1] = CALIB_VALUE_DEFAULT_SUB_MIN;

	for(uint32_t i=0; i<SENSOR_NUM; i++){
		c_state.middle[i] = (c_state.max[i] + c_state.min[i]) / 2;
		c_state.amplitude[i] = (c_state.max[i] - c_state.min[i]) / 2;
	}

	c_state.delta = CALIB_VALUE_DEFAULT_DELTA;
	c_state.alpha = CALIB_VALUE_DEFAULT_ALPHA;
	c_state.beta = CALIB_VALUE_DEFAULT_BETA;

	//saved calibration data
	calib_dat.chk_num = 0x12121212;
}

void cpCalcLength(uint16_t main, uint16_t sub)
{

	float sin_theta, cos_theta;
	float theta;
	float acc_length;

	//Judge Status A to D
	if(main >= c_state.middle[0]){
		if(sub >= c_state.middle[1]){
			c_state.sstat = STAT_A;
		}else{
			c_state.sstat = STAT_B;
		}
	}else{
		if(sub>=c_state.middle[1]){
			c_state.sstat = STAT_D;
		}else{
			c_state.sstat = STAT_C;
		}
	}

	//Calculate length
	switch(c_state.sstat){

		case STAT_A:
			if(STAT_D == c_state.sstat_prev){
				c_state.length += LENGTH_QUARTER;
			}else if(STAT_B == c_state.sstat_prev){
				c_state.length -= LENGTH_QUARTER;
			}
			break;

		case STAT_B:
			if(STAT_A == c_state.sstat_prev){
				c_state.length += LENGTH_QUARTER;
			}else if(STAT_C == c_state.sstat_prev){
				c_state.length -= LENGTH_QUARTER;
			}
			break;

		case STAT_C:
			if(STAT_B == c_state.sstat_prev){
				c_state.length += LENGTH_QUARTER;
			}else if(STAT_D == c_state.sstat_prev){
				c_state.length -= LENGTH_QUARTER;
			}
			break;

		case STAT_D:
			if(STAT_C == c_state.sstat_prev){
				c_state.length += LENGTH_QUARTER;
			}else if(STAT_A == c_state.sstat_prev){
				c_state.length -= LENGTH_QUARTER;
			}
			break;

		case STAT_INIT:
		default:
			break;
	}
	c_state.sstat_prev = c_state.sstat;


	//Calculate accurate length
	sin_theta = (float)((int32_t)main - c_state.middle[0]) / c_state.amplitude[0];
	cos_theta = (float)((int32_t)sub - c_state.middle[1]) / c_state.amplitude[1];
	theta = atan2f( (float)sin_theta, (float)cos_theta);
	acc_length = LENGTH_PERIOD * theta / (M_PI * 2);
	switch(c_state.sstat){

		case STAT_B:
			acc_length = acc_length - (LENGTH_QUARTER);
			break;

		case STAT_C:
			acc_length = acc_length + (LENGTH_QUARTER * 2);
			break;

		case STAT_D:
			acc_length = acc_length + (LENGTH_QUARTER);
			break;

		case STAT_A:
		case STAT_INIT:
		default:
			break;
	}

	c_state.accurate_length = c_state.length + acc_length;

}

CalibStatus cpCalibration(uint16_t y[SENSOR_NUM])
{
	float delta;
	int32_t y_offsetted[2];


	switch(c_state.cstat){

		case CALIB_INIT:
			for(uint32_t i=0; i<SENSOR_NUM; i++){
				maxmin_wave_cnt[i] = 0;
			}
			delta_cnt = 0;
			delta_sum = 0;

			c_state.cstat = CALIB_MAXMIN;
			break;

		case CALIB_MAXMIN:
			//count max-min wave.
			//When CALIB_MAXMIN_MIN => CALIB_MAXMIN_MAX, counter is increment.
			for(uint32_t i=0; i<SENSOR_NUM; i++){
				switch(c_state.cmaxmin[i]){
					case MAXMIN_INIT:
						if(y[i] >= (c_state.middle[i] + (c_state.amplitude[i] / 4) )){
							c_state.cmaxmin[i] = MAXMIN_MAX;
						}else if(y[i] < (c_state.middle[i] - (c_state.amplitude[i] / 4) )){
							c_state.cmaxmin[i] = MAXMIN_MIN;
						}
						break;

					case MAXMIN_MAX:
						if(y[i] < (c_state.middle[i] - (c_state.amplitude[i] / 4) )){
							c_state.cmaxmin[i] = MAXMIN_MIN;
						}
						break;

					case MAXMIN_MIN:
						if(y[i] >= (c_state.middle[i] + (c_state.amplitude[i] / 4) )){
							c_state.cmaxmin[i] = MAXMIN_MAX;
							maxmin_wave_cnt[i]++;
						}
						break;

					default:
						break;
				}

				if(y[i] >= c_state.max[i]){
					c_state.max[i] = y[i];
				}
				if(y[i] <= c_state.min[i]){
					c_state.min[i] = y[i];
				}
			}

			if(CALIB_MAXMIN_WAVE_NUM <= maxmin_wave_cnt[0] &&  CALIB_MAXMIN_WAVE_NUM <= maxmin_wave_cnt[1]){

				//calc zero point
				for(uint32_t i=0; i<SENSOR_NUM; i++){
					c_state.middle[i] = (c_state.max[i] + c_state.min[i]) / 2;
					c_state.amplitude[i] = (c_state.max[i] - c_state.min[i]) / 2;

					if(0 == c_state.middle[i] || 0 == c_state.amplitude[i]){
						return CALIB_NG;
					}
				}
				c_state.cstat = CALIB_PHASE;
			}
			break;

		case CALIB_PHASE:
			for(uint32_t i=0; i<SENSOR_NUM; i++){
				y_offsetted[i] = (int32_t)y[i] - (int32_t)c_state.middle[i];
			}


			if(CALIB_DELTA_MAX_THR >= y_offsetted[0] && y_offsetted[0] > CALIB_DELTA_MIN_THR && y_offsetted[1] > CALIB_DELTA_MIN_THR){
				delta = acosf((float)y_offsetted[1] / (float)c_state.amplitude[1]) - asinf((float)y_offsetted[0] / (float)c_state.amplitude[0]);
				delta_sum += delta;
				delta_cnt++;

				if(isinff(delta_sum) || isnanf(delta_sum)){
					c_state.cstat = CALIB_NG;
					break;
				}

				if(CALIB_DELTA_MEASURE_NUM <= delta_cnt){
					//store structure
					c_state.delta = delta_sum / (float)CALIB_DELTA_MEASURE_NUM;
					c_state.alpha = (float)c_state.amplitude[1] * cosf(c_state.delta) / c_state.amplitude[0];
					c_state.beta = (float)c_state.amplitude[1] * sinf(c_state.delta) / c_state.amplitude[0];
					c_state.cstat = CALIB_FINISHED;

					for(uint32_t i=0; i<SENSOR_NUM; i++){
						calib_dat.amplitude[i] = c_state.amplitude[i];
						calib_dat.max[i] = c_state.max[i];
						calib_dat.middle[i] = c_state.middle[i];
						calib_dat.min[i] = c_state.min[i];
					}
					calib_dat.delta = c_state.delta;
					calib_dat.chk_num = CALIB_DATA_CHECK_NUM;
				}

			}

			break;

		case CALIB_FINISHED:
			break;

		case CALIB_NG:
		default:
			break;
	}


	return c_state.cstat;



}

void cpCalibrationStart()
{
	c_state.smode = SCALE_CALIB;
	c_state.cstat = CALIB_INIT;
	for(uint32_t i=0; i<SENSOR_NUM; i++){
		c_state.cmaxmin[i] = MAXMIN_INIT;
		c_state.wave_cnt[i] = 0;
	}
}


void cpApplyPhaseCompensation(uint16_t y[SENSOR_NUM])
{
#ifdef CFG_COMPENSATION_ON
	int32_t y_offsetted[2];

	float cos_delta_inv, tan_delta, k2_k1;
	float a1, a2, ret;

	for(uint32_t i=0; i<SENSOR_NUM; i++){
		y_offsetted[i] = (int32_t)y[i] - (int32_t)c_state.middle[i];
	}

	cos_delta_inv = 1.0f / cosf(c_state.delta);
	tan_delta = tanf(c_state.delta);
	k2_k1 = (float)c_state.amplitude[1] / (float)c_state.amplitude[0];

	a1 = y_offsetted[1] * cos_delta_inv;
	a2 = y_offsetted[0] * k2_k1 * tan_delta;

	ret = a1 + a2 + c_state.middle[1];
	y[1] = ret;
#endif
}

inline float cpGetLength()
{
	return c_state.length;
}

inline float cpGetAccLength()
{
	return c_state.accurate_length - c_state.zero_pos_length;
}

inline void cpSetZeroPosition()
{
	c_state.zero_pos_length = c_state.accurate_length;
}

inline ScaleMode cpGetCaliperMode()
{
	return c_state.smode;
}

inline void cpSetCaliperMode(ScaleMode smd)
{
	c_state.smode = smd;
}
