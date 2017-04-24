#include "coordinate.h"
using namespace std;
void STATE::KalmanStateUpdate (const INPUT* Input) 
{	
	last_state = *this;
	x = x + Input->GetTranslation () * cos (orientation + Input->GetAngle () / 2);
	y = y + Input->GetTranslation () * sin (orientation + Input->GetAngle () / 2);
	orientation = orientation + Input->GetAngle ();
}

void STATE::KalmanStateUpdate(const double x, const double y, const double z, const double orientation)
{
    last_state = *this;
	STATE::x += x;
	STATE::y += y;
	STATE::z += z;
	STATE::orientation += orientation;
}
void STATE::KalmanCovUpdate (const INPUT* Input, MATRIX* update_uncertainty) 
{
	MATRIX Phi (3, 3);

	double* P = Phi.GetHead ();
	MATRIX temp_1 (3, 3);
	//phi = [ 1, 0, -delta(1) * sin( r_post(3) + delta(2) / 2 );
    //        0, 1, delta(1) * cos( r_post(3) + delta(2) / 2 );
    //        0, 0, 1                                            ];  
	P[0] = P[4] = P[8] = 1;
	P[1] = P[3] = P[6] = P[7] = 0;
	P[2] = - Input->GetTranslation () * sin (orientation + Input->GetAngle () / 2);
	P[5] = Input->GetTranslation () * cos (orientation + Input->GetAngle () / 2);
	MATRIX Phi_trans (Phi.trsp ());
	//y = phi * P_post * phi' + Q;
	Phi.multiple (cov, temp_1);
	temp_1.multiple (Phi_trans, cov);
	cov.add(*update_uncertainty,cov);
}

MATRIX STATE::KalmanGain(MATRIX* obs_unc, MATRIX* obs_matrix) 
{
	MATRIX obs_matrix_trans (obs_matrix->trsp ());
	MATRIX temp_1 (cov.GetLength (), obs_matrix_trans.GetWidth ());
	//K = P_apr * H' * ( H * P_apr * H'+ R ) ^ ( - 1 );
	MATRIX temp_2 (obs_matrix->GetLength (), obs_matrix_trans.GetWidth ());
	MATRIX temp_3 (obs_unc->GetLength (), obs_unc->GetWidth ());
	MATRIX kalman_gain (cov.GetLength (), obs_matrix_trans.GetWidth ());
	//if (obs_unc->GetWidth () != obs_unc->GetLength ())
		//cout << "wrong matrix!" << endl;
	cov.multiple (obs_matrix_trans, temp_1); // P_apr * H'
	obs_matrix->multiple (temp_1, temp_2); // H * P_apr * H'
	temp_2.add (*obs_unc, temp_3);
	temp_3.inv (temp_3); // ( H * P_apr * H'+ R ) ^ ( - 1 )
	temp_1.multiple (temp_3, kalman_gain); // P_apr * H' * ( H * P_apr * H'+ R ) ^ ( - 1 )
	return kalman_gain;
}

void STATE::KalmanStateCorrection(MATRIX* obs, MATRIX* kalman_gain, MATRIX* obs_matrix, double obs_z)
{
	
	double vector_state[3] = {x, y, orientation};	
	MATRIX state (vector_state, 3, 1); 
	MATRIX temp_1 (obs_matrix->GetLength (), state.GetWidth ());
	MATRIX temp_2 (kalman_gain->GetLength (), obs->GetWidth ());
	//if (obs->GetLength () != temp_1.GetLength ()
		//|| obs->GetWidth () != temp_1.GetWidth ())
		//cout << "Matrix operation error!" << endl;
	//if (kalman_gain->GetWidth () != temp_1.GetLength ())
		//cout << "Matrix operation error!" << endl;
	//r_posteriori = r_apriori + K * ( z - H * r_apriori );
	obs_matrix->multiple (state, temp_1); // H * r_apriori 
	obs->sub (temp_1, temp_1); // z - H * r_apriori
	kalman_gain->multiple (temp_1, temp_2); //K * ( z - H * r_apriori )
	state.add (temp_2, state); // r_apriori + K * ( z - H * r_apriori )
	x = state.GetHead ()[0];
	y = state.GetHead ()[1];
	z = obs_z;
	orientation = state.GetHead ()[2];
}

void STATE::KalmanStateCorrection_abs(MATRIX* obs, MATRIX* kalman_gain, MATRIX* obs_matrix, double obs_z, double orientation)
{
	
	double vector_state[2] = {x, y};	
	MATRIX state (vector_state, 2, 1); 
	MATRIX temp_1 (obs_matrix->GetLength (), state.GetWidth ());
	MATRIX temp_2 (kalman_gain->GetLength (), obs->GetWidth ());
	/*if (obs->GetLength () != temp_1.GetLength ()
		|| obs->GetWidth () != temp_1.GetWidth ())
		cout << "Matrix operation error!" << endl;
	if (kalman_gain->GetWidth () != temp_1.GetLength ())
		cout << "Matrix operation error!" << endl;*/
	//r_posteriori = r_apriori + K * ( z - H * r_apriori );
	obs_matrix->multiple (state, temp_1); // H * r_apriori 
	obs->sub (temp_1, temp_1); // z - H * r_apriori
	kalman_gain->multiple (temp_1, temp_2); //K * ( z - H * r_apriori )
	state.add (temp_2, state); // r_apriori + K * ( z - H * r_apriori )
	x = state.GetHead ()[0];
	y = state.GetHead ()[1];
	z = obs_z;
	STATE::orientation = orientation;
}

void STATE::KalmanCovCorrecction(MATRIX* obs_matrix, MATRIX* kalman_gain)
{
	//y = ( v - K * H ) * P_apr;
	double vector_unit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	MATRIX unit (vector_unit, 3, 3);	
	MATRIX temp_1 (kalman_gain->GetLength (), obs_matrix->GetWidth ());
	MATRIX temp_2 (kalman_gain->GetLength (), cov.GetWidth ());
	kalman_gain->multiple (*obs_matrix, temp_1); // K * H 
	unit.sub (temp_1, temp_1); //  v - K * H 
	temp_1.multiple (cov, temp_2);
	cov = temp_2;
}
void STATE::KalmanCovCorrecction_abs(MATRIX* obs_matrix, MATRIX* kalman_gain)
{
	//y = ( v - K * H ) * P_apr;
	double vector_unit[9] = {1, 0, 0, 1};
	MATRIX unit (vector_unit, 2, 2);	
	MATRIX temp_1 (kalman_gain->GetLength (), obs_matrix->GetWidth ());
	MATRIX temp_2 (kalman_gain->GetLength (), cov.GetWidth ());
	kalman_gain->multiple (*obs_matrix, temp_1); // K * H 
	unit.sub (temp_1, temp_1); //  v - K * H 
	temp_1.multiple (cov, temp_2);
	cov = temp_2;
}
