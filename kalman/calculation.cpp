#include "coordinate.h"
extern RECEIVER_TABLE table;
extern PACKAGE_MULTI package_multi;
extern PACKAGE_SINGLE package_single;

/*调用绝对定位函数说明 Jan. 19th 2014 by David*/
/*Newton: 采用牛顿迭代法求解非线性方程，收敛条件为系数矩阵非奇异，给定初值，经过几轮迭代后即可收敛，精度较高；其中，收到四个
超声波信号的时候X,Y,Z坐标绝对收敛，收敛结果与初值无关；收到三个超声波信号的时候保证迭代初值的Z坐标小于超声波接收节
点安装位置高度，X,Y坐标绝对收敛；收到两个超声波信号的时候保证迭代初值X,Y位于合适范围内（在一个平面上，（X,Y）所代
表的坐标点位于接收到的两个超声波结点坐标连线包含移动目标真实位置的一侧），Z经过迭代后不发生改变，保持进入迭代时的初值，
故Z需要预先测量。
	INPUT:	int						round			迭代次数，一般10次以上就能保证精确收敛；
	OUTPUT: void									New函数自动改写p中用于记录移动目标位置估计的成员coordinate		*/

void PACKAGE_MULTI::Newton (int round)
{
	int i, n;
	double x_temp, y_temp, z_temp;
	MATRIX coef(size, size),
		   A_inv(size, size ),
		   B(size, 1 ), 
		   U(size, 1 );
	double* A = coef.GetHead ();
	double* b = B.GetHead ();
	double* u = U.GetHead ();
	for ( n = 0; n < round; n ++ ){
		switch (size) {
		case 2: 			
			A[0] = 2 * (init.GetX () - node[0].GetX ());
			A[1] = 2 * (init.GetY () - node[0].GetY ());
			A[2] = 2 * (init.GetX () - node[1].GetX ());
			A[3] = 2 * (init.GetY () - node[1].GetY ());

			z_temp = init.GetZ () - node[0].GetZ ();
			b[0] = - pow (A[0] / 2, 2) - pow (A[1] / 2, 2) + pow (dist[0], 2) - pow (z_temp, 2);
			z_temp = init.GetZ () - node[1].GetZ ();
			b[1] = - pow (A[2] / 2, 2) - pow (A[3] / 2, 2) + pow (dist[1], 2) - pow (z_temp, 2);
			break;
		case 3: 
			for ( i = 0; i < 3; i ++ ){
				A[i * 3 + 0] = 2 * (init.GetX () - node[i].GetX ());
				A[i * 3 + 1] = 2 * (init.GetY () - node[i].GetY ());
				A[i * 3 + 2] = 2 * (init.GetZ () - node[i].GetZ ());
				b[i] = - pow (A[i * 3 + 0] / 2,2) - pow (A[i * 3 + 1] / 2, 2)
				- pow (A[i * 3 + 2] / 2, 2) + pow (dist[i], 2);
			}
			break;
		}
		coef.inv ( A_inv );
		A_inv.multiple ( B, U );
		switch (size) {
		case 2:		
			x_temp = init.GetX () + u[0];
			y_temp = init.GetY () + u[1];
			z_temp = init.GetZ ();
			init.Update (x_temp, y_temp, z_temp);
			break;
		case 3:
			x_temp = init.GetX () + u[0];
			y_temp = init.GetY () + u[1];
			z_temp = init.GetZ () + u[2];
			init.Update (x_temp, y_temp, z_temp);
			break;
		}
	}
}

/*调用绝对定位函数说明 Jan. 20th 2014 by David*/
/*UltraLocalization_1 函数说明
	当只有收到一个超声波信号的时候，并不是第一次收到距离就可以立即定位。没收到一次信号，需要向二类参数包PACKAGE_SINGLE p2 
加入一组数据。加入的数据格式为：当前收到与超声波结点的距离distance[0]以及与上一时刻的相对位移d（由其他定位方式得到），
p2中有一个循环队列，设第k时刻接收到的这样一组数据(distance[0]k,dk)，则向队列中加入该对数据。队列的大小由coordinate.h中的宏定义
MAX_PACKAGE_SIZE_2确定。p2.front指向下一个需要加入数据的队列位置，当记录的总共条目大于MAX_PACKAGE_SIZE_2，front指向链表表头
从头开始记录。当连续接收到的数据大于coordinate.h中的宏定义INCREMENT，即IsFull函数返回值为真，UltraLocalization_1开始第一次定位。
根据之前的经验，INCREMENT可以根据需要设定。如果INCREMENT增大的，单超声波定位的精度收到相对误差d和距离distance的影响较小，但是这样
定位的反应也相对较慢。反之，定位精度对于误差敏感度较大，容易收到d和distance的误差影响。
	GetSub函数用于从队列中提取相距INCREMENT的两个条目中的数据输入到Newton中进行计算。每一次提取的条目较上一次向右移动一个条目距离。
	当接收到的超声波ID改变的时候，即IsSame函数判定返回为假，队列清空，front重置。所以，必须保证采用单超声波定位的时候，队列中的数据
全部为从一个超声波接收点得到的数据。如果某一时刻突然接收到了两个及以上的超声波数据，则队列也清空，front重置，放弃之前累积的数据，立即
采用两个超声波信号定位的函数，因为我们认为那样精度更高。
	INPUT:	void
	OUTPUT: int			返回0表示成功，-1表示数据不可用，采用直接加上位移的策略									*/
int PACKAGE_SINGLE::UltraLocalization ()
{
	PACKAGE_MULTI subp;
	//如果获得子包成功，则直接用获得的子包计算机器人的当前坐标
	if (GetSub (subp)) {
		subp.Newton (ITERATION_NUMBER);
		if (correction_count < CORRECTION_COUNT || subp.EuclideanFilter (init)) {
			init = subp.GetCoordinate (); // 只有当通过滤波才修改值，否则本次计算没有任何效果
			correction_count ++;
			return 0;
		}
		else return -1;
	}
	//若果获得子包失败，直接返回-1
	else return -1;
}

/*调用绝对定位函数说明 Jan. 20th 2014 by David*/
/*UltraLocalization_2 函数说明
	用于当只有两个超声波信号时候定位。函数调用方式较简单，下面的三个超声波信号调用方式也相同，不在赘述。
*/
int PACKAGE_MULTI::UltraLocalization ()
{
	PACKAGE_MULTI subp[4];
	int subnum;
	double x_temp, y_temp, z_temp;
	COORDINATE c[4], calc_coordinate(0, 0, 0);
	switch(size) {
	case 2:
		Newton (ITERATION_NUMBER);
		return 0;
	case 3:
		Newton (ITERATION_NUMBER);
		return 0;
	case 4:
		//用于当只有四个超声波信号时候定位。实质是从四个数据中提取出四组每组只有三个数据的数据包，
		//对每三个数据的数据包采用普通的三超声波定位，计算结果求平均。日后也可考虑用最小二乘等
		//其他方式进行数据耦合。
		 if ((subnum = GetSub (subp)) != 0) {
			for (int i = 0; i < subnum; i ++) {
				subp[i].UltraLocalization ();
				c[i] = subp[i].init;
				x_temp = calc_coordinate.GetX () + c[i].GetX ();
				y_temp = calc_coordinate.GetY () + c[i].GetY ();
				z_temp = calc_coordinate.GetZ () + c[i].GetZ ();
				calc_coordinate.Update (x_temp, y_temp, z_temp);
			}	
			x_temp = calc_coordinate.GetX ( ) / subnum;
			y_temp = calc_coordinate.GetY ( ) / subnum;
			z_temp = calc_coordinate.GetZ ( ) / subnum;
			calc_coordinate.Update (x_temp, y_temp, z_temp);
            init = calc_coordinate;
			return 0;
		}		
        else
			return -1;
	default:
		return -1;
	}
}

/*调用绝对定位函数说明 Jan. 19th 2014 by David*/
/*Calculation 函数说明（实际调用时参照main.cpp函数调用方法）
对于收到一、二、三、四个超声波信号的情况，函数调用形式是一样的。函数会自动根据 size 大小选择相应的计算程序
	INPUT:	int			size			接收到的超声波信号数量；
			double*		distance		接收到的距离数量，数组大小为size，仅包含有效的距离数据；
			int*		ID				接收到的超声波ID，数组大小为size，且顺序与distance[]一一对应；
			int*		group			接收到的超声波组号，要求当前所有超声波属于一组；
			COORDINATE	&c				当前位置估计引用；
			COORDINATE	&rel_c			局部坐标系更新；
			DELTA		&d				相邻测量时刻间发生的相对位移，由其他定位方式获得，用于只有一个超声波信号的情况；
	OUTPUT: int							计算成功返回0，否则返回-1										*/

int Calculation (const int size, const double* distance, const int* ID, 
				 const int* group, COORDINATE& abs_c, const COORDINATE &rel_c, MATRIX& Cov)
{
	int ID_abs[4];
	RECEIVER_NODE* node_temp = new RECEIVER_NODE[4] ;
	//因为超声波信号可能来自不同的组
	//ID_abs用来记录一组超声波的绝对ID
	//node_temp用来记录一组节点
	for (int i = 0; i < size; i ++) {
		ID_abs[i] = group[i] + ID[i];
		node_temp[i] = table.GetNode (ID_abs[i]);
	}
	switch (size) {
	case 1:
		package_single.SetCoordinate (abs_c);
		//先判断超声波节点是否改变，如果改变重置table
		if (!package_single.IsDifferent (ID_abs[0]))
			package_single.Update (rel_c, distance[0]);
		else {
			package_single.Reset (&node_temp[0]);
			package_single.Update (rel_c, distance[0]);
		}
		if (package_single.IsEnough ()) {
			// 只有当数量足够的时候才进入定位算法
			if(package_single.UltraLocalization () == 0) {
			//计算成功才会修改init，后由GetCoordinate传递给abs_c
				abs_c = package_single.GetCoordinate();
				Cov = package_single.UncertaintyEvaluation ();
				return 0;
			}
			else return -1;
		}
		else return -1;
	case 2: 
		package_multi.Update (distance, 2, node_temp, &abs_c);
		break;
	case 3:
		package_multi.Update (distance, 3, node_temp, &abs_c);
		break;		
	case 4:
		package_multi.Update (distance, 4, node_temp, &abs_c);
		break;
	default:
		return -1;
	}
	package_single.correction_count = CORRECTION_COUNT; // 说明Euclidean Filter这个时候可以发挥作用了！
	package_multi.UltraLocalization ();
	abs_c = package_multi.GetCoordinate ();
	Cov.update (multiultra_error); //这里是多超声波的定位误差矩阵，主要是看超声波测距的精度，先暂时给一个比较方便
									//以后可以改
	return 0;
}
