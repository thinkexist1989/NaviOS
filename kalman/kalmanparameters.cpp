#include "matrix.h"

const double initial_abs_state_uncertainty[9] = {1000 * 1000, 0,		
											     0,			  1000 * 1000};
MATRIX abs_cov(initial_abs_state_uncertainty, 2, 2);

const double initial_rel_state_uncertainty[9] = {100,		0,		0,
												 0,		100,		0,
												 0,		0,		4 * 3.141592654 * 3.141592654};
MATRIX rel_cov(initial_rel_state_uncertainty, 3, 3);

const double vector_rel_state_update_uncertainty[9] = {0.494753865,	0,			0, 
												   0,			0.494753865,	0, 
												   0,			0,			0.00003196};
MATRIX state_rel_update_uncertainty(vector_rel_state_update_uncertainty, 3, 3);

const double vector_abs_state_update_uncertainty[4] = {0.494753865,	0, 
														0,			0.494753865};
MATRIX state_abs_update_uncertainty(vector_abs_state_update_uncertainty, 2, 2);

const double orien_uncertainty = 0.0076;
const double dist_uncertainty = 133.62792;
MATRIX rel_obs_uncertainty (&orien_uncertainty, 1, 1);

const double vector_rel_obs[3] = {0, 0, 1};
MATRIX obs_rel_matrix (vector_rel_obs, 1, 3);

const double vector_abs_obs[4] = {1, 0, 
								  0, 1};
MATRIX obs_abs_matrix (vector_abs_obs, 2, 2);

	// error = diag([ error_src(1) ^ 2,     error_src(1) ^ 2, ...
      //             error_src(2) ^ 2 * N, error_src(2) ^ 2 * N ]);
const double vector_error[16] = {dist_uncertainty,	0,	0,	0,
								 0,	dist_uncertainty, 0,	0,
								 0,	0, 0.4947538 * 20, 0,
								 0, 0, 0, 0.4947538 * 20};
MATRIX error (vector_error, 4, 4);

const double vector_multiultra_error[4] = {400, 0,
											0, 400};
MATRIX multiultra_error(vector_multiultra_error, 2, 2);
