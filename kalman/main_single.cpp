#include "coordinate.h"
#define SAMPLE 100
using namespace std;
RECEIVER_TABLE table;
PACKAGE_SINGLE package_single;
PACKAGE_MULTI package_multi;
COORDINATE c (500, 100, 0);
STATE state_abs (100, 100, 0, 0, abs_cov);
STATE state_rel (0, 0, 0, 0, rel_cov);

INPUT input;
void DataRead (double* left_wheel, double* right_wheel, double* dist, double* orientation) 
{
	ifstream input;
	input.open ("E:\\Projects\\Graduate Design\\Mfile\\test.txt");
	double temp_left, temp_right, temp_dist, temp_orientation;
	input >> temp_left >> temp_right >> temp_dist >> temp_orientation;
	for (int i = 0; i < SAMPLE; i ++) {
		input >> temp_left >> temp_right >> temp_dist >> temp_orientation;
		//left_wheel[i] = temp_left + 10;
		//right_wheel[i] = temp_right + 10;
		left_wheel[i] = 10;
		right_wheel[i] = 10;
		dist[i] = temp_dist;
		orientation[i] = temp_orientation;
	}
}
int main ()
{
	int size = 1;
	int ID[3] = {10, 1, 2};
	int group = 0;
	COORDINATE abs_obs;
	COORDINATE delta;
	double vector_obs_abs[3];
	MATRIX R_obs (2, 2);
	MATRIX kalman_gain_rel (3, 1);
	MATRIX kalman_gain_abs (3, 3);
	double left_wheel[SAMPLE], right_wheel[SAMPLE], dist[SAMPLE], orientation[SAMPLE];
	DataRead (left_wheel, right_wheel, dist, orientation);
	for (int i = 0; i < SAMPLE; i ++) {
		if (i == INCREMENT - 2)
			i = i;
		/*每次数据一来，不管是多少个超声波的数据（至少是一个以上），首先更新state_rel，也就是局部
		  机器人坐标，通过state_rel来维护局部机器人坐标的表，*/
		input.Reload(left_wheel[i], right_wheel[i]); 
		// 底层卡尔曼滤波更新
		state_rel.KalmanStateUpdate (&input);
		state_rel.KalmanCovUpdate (&input, &state_rel_update_uncertainty);
		kalman_gain_rel = state_rel.KalmanGain (&rel_obs_uncertainty, &obs_rel_matrix);
		state_rel.KalmanStateCorrection (&MATRIX (&orientation[i], 1, 1), 
										 &kalman_gain_rel, 
										 &obs_rel_matrix, 
										 0);
		state_rel.KalmanCovCorrecction (&obs_rel_matrix, &kalman_gain_rel);
		// 顶层卡尔曼滤波更新
		delta = state_rel.GetDelta ();
		state_abs.KalmanStateUpdate (delta.GetX (), delta.GetY (), 0, 0);		
		state_abs.KalmanCovUpdate (&state_abs_update_uncertainty);
		abs_obs = state_abs; // 用观察坐标将最当前state_abs的坐标状态传递进Calculation函数
		if (Calculation (size, &dist[i], ID, &group, abs_obs, state_rel,  R_obs) == 0) {
		// 函数计算后将abs_obs修改为计算得到的坐标，R_obs为本次计算的置信度
			kalman_gain_abs = state_abs.KalmanGain (&R_obs, &obs_abs_matrix);
			vector_obs_abs[0] = abs_obs.GetX ();
			vector_obs_abs[1] = abs_obs.GetY ();
			state_abs.KalmanStateCorrection_abs (&MATRIX (vector_obs_abs, 2, 1),
												&kalman_gain_abs,
												&obs_abs_matrix,
												abs_obs.GetZ (),
												orientation[i]);
			state_abs.KalmanCovCorrecction_abs (&obs_abs_matrix, &kalman_gain_abs);
		}
	}
	return 0;
}