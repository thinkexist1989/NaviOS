#include "fstream"
#include <math.h>
#include <stdlib.h>
#include "iostream"
#include "string"
#include "matrix.h"

#ifndef COORDINATE_H
#define COORDINATE_H
class COORDINATE;

extern MATRIX abs_cov;
extern MATRIX rel_cov;
extern MATRIX state_rel_update_uncertainty;
extern MATRIX state_abs_update_uncertainty;
extern MATRIX obs_rel_matrix;
extern MATRIX rel_obs_uncertainty;
extern MATRIX error;
extern MATRIX multiultra_error;
extern MATRIX obs_abs_matrix;

extern int Calculation (const int size, const double* distance, const int* ID, 
						const int* group, COORDINATE& c, const COORDINATE &rel_c, MATRIX& Cov);
extern const double dist_uncertainty;
#define WHEEL_DISTANCE 340
#define MAX_RECEIVER_TABLE_SIZE 12
#define MAX_PACKAGE_SINGLE_TABLE_SIZE 100
//INCREMENT用来决定位移矢量的长度，为从TABLE中用来采样的
#define INCREMENT 100
#define ERROR_DIST 1
#define ERROR_TRAN 1
#define EUCLIDEAN_THRESHOLD	100
#define ITERATION_NUMBER 20
#define CORRECTION_COUNT 10
#define PI 3.14159
#define SOUND_SPEED 340000  //mm/s
//THRESHOLD_ANGLE用来规定最小夹角的门限,一般认为如果机器人与超声波节点连线和
//位移矢量夹角小于THRESHOLD_ANGLE,则估计结果会因为矩阵t
#define THRESHOLD_ANGLE 20

typedef int MODE;
class COVARIANCE : public MATRIX 
{

};
// Standard Coordinate Data Structure
class COORDINATE {
public:
	COORDINATE ();//default constructor
	COORDINATE (const double, const double, double);//Normal constructor
	COORDINATE (const COORDINATE &c);//Copy constructor
	COORDINATE& operator = (const COORDINATE&);
    const COORDINATE& operator - (const COORDINATE&);
	virtual ~COORDINATE () {};
	void virtual Update (const COORDINATE &c);
	void virtual Update (double X, double Y, double Z);
	bool Collinear (const COORDINATE c1, const COORDINATE c2) const;
	double GetX () const;
	double GetY () const;
	double GetZ () const;
protected:
	double		x;
	double		y;
	double		z;
};

class INPUT {
private:
	double angle;
	double translation[2]; // translation[0] is left wheel; [1] is right wheel
	double delta_trans;
	double delta_angle;
public:
	INPUT ();
	void Reload (double, double);
	double GetAngle () const;
	double GetTranslation () const;
};

class STATE: public COORDINATE
{
public:
	STATE (double X, double Y, double Z, double Orient, MATRIX Cov) 
        : COORDINATE (X, Y, Z), cov(Cov) { orientation = Orient; last_state.Update (X, Y, Z); };
	~STATE () {};
	void		KalmanStateUpdate(const INPUT*);
	void		KalmanStateUpdate(const double x, const double y, const double z, const double orientation);
	void		KalmanCovUpdate(const INPUT*, MATRIX* update_uncertainty);
	void		KalmanCovUpdate(MATRIX* update_uncertainty) { cov.add (*update_uncertainty, cov); };
	MATRIX		KalmanGain(MATRIX* observation_uncertainty, MATRIX* obs_matrix);
	void		KalmanStateCorrection(MATRIX* obs, MATRIX* kalman_gain, MATRIX* obs_matrix, double obs_z);
	void		KalmanStateCorrection_abs(MATRIX* obs, MATRIX* kalman_gain, MATRIX* obs_matrix, double obs_z, double orientation);
	void		KalmanCovCorrecction(MATRIX* obs_matrix, MATRIX* kalman_gain);
	void		KalmanCovCorrecction_abs(MATRIX* obs_matrix, MATRIX* kalman_gain);
	double		GetOrientation() const { return orientation; };
	MATRIX*		GetUncertainty () { return &cov; };
    COORDINATE	GetDelta () const { return COORDINATE(x - last_state.GetX (),y - last_state.GetY (), z - last_state.GetZ ()); };

    COORDINATE	last_state;
protected:	
	MATRIX		cov;
    double		orientation;
};
// Receiver Position Node
class RECEIVER_NODE : public COORDINATE
{
public:
	RECEIVER_NODE ();
	RECEIVER_NODE (const RECEIVER_NODE& r);	
	~RECEIVER_NODE ();
	RECEIVER_NODE&	operator = (const RECEIVER_NODE&);
	void			Update (COORDINATE c, int ID, double ks);
	int				GetID () const;
	COORDINATE		GetCoordinate () const;
private:
	int				ID; //这里给的ID都是绝对ID编号，ID = group * 4 + ID_rel
	double			ks; // Correction Parameter
};

class RECEIVER_TABLE {
protected:
	RECEIVER_NODE node[MAX_RECEIVER_TABLE_SIZE];
	/*以超声波节点ID作为关键词搜索RECEIVER_TABLE()*/
	int node_count;
public:
	RECEIVER_NODE& GetNode (const int Iterator);
	int Search (int key) const;
	/*初始化包括各超声波接收节点的安装位置坐标 NODE_X，NODE_Y，NODE_Z ，测距修正*/

	RECEIVER_TABLE ();
	~RECEIVER_TABLE ();
};

class RECEIVE_PACKAGE {
public:
	RECEIVE_PACKAGE ();
	RECEIVE_PACKAGE (const RECEIVE_PACKAGE&);
	virtual ~RECEIVE_PACKAGE () { delete []node; };
	void			Update (const int Size, const RECEIVER_NODE* Node, 
							const COORDINATE* Coordinate);
	COORDINATE		GetCoordinate () const { return init; };
	void			SetCoordinate (COORDINATE c) { init = c; };
	int				virtual UltraLocalization() = 0;
	bool			EuclideanFilter (COORDINATE c) const;
protected:
	int				size;
	int				group;
	COORDINATE		init;
	RECEIVER_NODE*	node;
};
// Received Package Used by 2~4 Rangers Localization
class PACKAGE_MULTI : public RECEIVE_PACKAGE 
{
public:	
	PACKAGE_MULTI ();
	PACKAGE_MULTI (const PACKAGE_MULTI& p);
	~PACKAGE_MULTI ();
	void			Newton (int round);
	int				GetSub (PACKAGE_MULTI* subp);	
	void			Update (const double* Dist, const int Size, 
							const RECEIVER_NODE* Node, const COORDINATE* Coordinate);
    int				UltraLocalization ();
protected:	
	double*			dist;
};

class PACKAGE_SINGLE :public RECEIVE_PACKAGE
{
private:
	COORDINATE*		list_local_coordinate;
	double*			list_dist;
	bool			full;
	int				front;	
public:
	PACKAGE_SINGLE ();
	~PACKAGE_SINGLE ();
	void			Update (const COORDINATE rel_c, const double Dist);
	void			Reset (const RECEIVER_NODE* Node);
	int				GetSub (PACKAGE_MULTI& subpackage);
	int				Decrement (int interval);
	bool			IsEnough ();
	bool			IsDifferent (const int ID);
	bool			IsUseful (int low, int high);
    int				UltraLocalization ();
	MATRIX			UncertaintyEvaluation ();	
	int				GetTop () const;
	int				correction_count;
};

#endif
