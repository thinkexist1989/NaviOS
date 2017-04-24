#include "coordinate.h"
extern RECEIVER_TABLE table;
extern PACKAGE_MULTI package_multi;
extern PACKAGE_SINGLE package_single;

/*���þ��Զ�λ����˵�� Jan. 19th 2014 by David*/
/*Newton: ����ţ�ٵ������������Է��̣���������Ϊϵ����������죬������ֵ���������ֵ����󼴿����������Ƚϸߣ����У��յ��ĸ�
�������źŵ�ʱ��X,Y,Z�����������������������ֵ�޹أ��յ������������źŵ�ʱ��֤������ֵ��Z����С�ڳ��������ս�
�㰲װλ�ø߶ȣ�X,Y��������������յ������������źŵ�ʱ��֤������ֵX,Yλ�ں��ʷ�Χ�ڣ���һ��ƽ���ϣ���X,Y������
��������λ�ڽ��յ�����������������������߰����ƶ�Ŀ����ʵλ�õ�һ�ࣩ��Z���������󲻷����ı䣬���ֽ������ʱ�ĳ�ֵ��
��Z��ҪԤ�Ȳ�����
	INPUT:	int						round			����������һ��10�����Ͼ��ܱ�֤��ȷ������
	OUTPUT: void									New�����Զ���дp�����ڼ�¼�ƶ�Ŀ��λ�ù��Ƶĳ�Աcoordinate		*/

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

/*���þ��Զ�λ����˵�� Jan. 20th 2014 by David*/
/*UltraLocalization_1 ����˵��
	��ֻ���յ�һ���������źŵ�ʱ�򣬲����ǵ�һ���յ�����Ϳ���������λ��û�յ�һ���źţ���Ҫ����������PACKAGE_SINGLE p2 
����һ�����ݡ���������ݸ�ʽΪ����ǰ�յ��볬�������ľ���distance[0]�Լ�����һʱ�̵����λ��d����������λ��ʽ�õ�����
p2����һ��ѭ�����У����kʱ�̽��յ�������һ������(distance[0]k,dk)����������м���ö����ݡ����еĴ�С��coordinate.h�еĺ궨��
MAX_PACKAGE_SIZE_2ȷ����p2.frontָ����һ����Ҫ�������ݵĶ���λ�ã�����¼���ܹ���Ŀ����MAX_PACKAGE_SIZE_2��frontָ�������ͷ
��ͷ��ʼ��¼�����������յ������ݴ���coordinate.h�еĺ궨��INCREMENT����IsFull��������ֵΪ�棬UltraLocalization_1��ʼ��һ�ζ�λ��
����֮ǰ�ľ��飬INCREMENT���Ը�����Ҫ�趨�����INCREMENT����ģ�����������λ�ľ����յ�������d�;���distance��Ӱ���С����������
��λ�ķ�ӦҲ��Խ�������֮����λ���ȶ���������жȽϴ������յ�d��distance�����Ӱ�졣
	GetSub�������ڴӶ�������ȡ���INCREMENT��������Ŀ�е��������뵽Newton�н��м��㡣ÿһ����ȡ����Ŀ����һ�������ƶ�һ����Ŀ���롣
	�����յ��ĳ�����ID�ı��ʱ�򣬼�IsSame�����ж�����Ϊ�٣�������գ�front���á����ԣ����뱣֤���õ���������λ��ʱ�򣬶����е�����
ȫ��Ϊ��һ�����������յ�õ������ݡ����ĳһʱ��ͻȻ���յ������������ϵĳ��������ݣ������Ҳ��գ�front���ã�����֮ǰ�ۻ������ݣ�����
���������������źŶ�λ�ĺ�������Ϊ������Ϊ�������ȸ��ߡ�
	INPUT:	void
	OUTPUT: int			����0��ʾ�ɹ���-1��ʾ���ݲ����ã�����ֱ�Ӽ���λ�ƵĲ���									*/
int PACKAGE_SINGLE::UltraLocalization ()
{
	PACKAGE_MULTI subp;
	//�������Ӱ��ɹ�����ֱ���û�õ��Ӱ���������˵ĵ�ǰ����
	if (GetSub (subp)) {
		subp.Newton (ITERATION_NUMBER);
		if (correction_count < CORRECTION_COUNT || subp.EuclideanFilter (init)) {
			init = subp.GetCoordinate (); // ֻ�е�ͨ���˲����޸�ֵ�����򱾴μ���û���κ�Ч��
			correction_count ++;
			return 0;
		}
		else return -1;
	}
	//��������Ӱ�ʧ�ܣ�ֱ�ӷ���-1
	else return -1;
}

/*���þ��Զ�λ����˵�� Jan. 20th 2014 by David*/
/*UltraLocalization_2 ����˵��
	���ڵ�ֻ�������������ź�ʱ��λ���������÷�ʽ�ϼ򵥣�����������������źŵ��÷�ʽҲ��ͬ������׸����
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
		//���ڵ�ֻ���ĸ��������ź�ʱ��λ��ʵ���Ǵ��ĸ���������ȡ������ÿ��ֻ���������ݵ����ݰ���
		//��ÿ�������ݵ����ݰ�������ͨ������������λ����������ƽ�����պ�Ҳ�ɿ�������С���˵�
		//������ʽ����������ϡ�
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

/*���þ��Զ�λ����˵�� Jan. 19th 2014 by David*/
/*Calculation ����˵����ʵ�ʵ���ʱ����main.cpp�������÷�����
�����յ�һ�����������ĸ��������źŵ����������������ʽ��һ���ġ��������Զ����� size ��Сѡ����Ӧ�ļ������
	INPUT:	int			size			���յ��ĳ������ź�������
			double*		distance		���յ��ľ��������������СΪsize����������Ч�ľ������ݣ�
			int*		ID				���յ��ĳ�����ID�������СΪsize����˳����distance[]һһ��Ӧ��
			int*		group			���յ��ĳ�������ţ�Ҫ��ǰ���г���������һ�飻
			COORDINATE	&c				��ǰλ�ù������ã�
			COORDINATE	&rel_c			�ֲ�����ϵ���£�
			DELTA		&d				���ڲ���ʱ�̼䷢�������λ�ƣ���������λ��ʽ��ã�����ֻ��һ���������źŵ������
	OUTPUT: int							����ɹ�����0�����򷵻�-1										*/

int Calculation (const int size, const double* distance, const int* ID, 
				 const int* group, COORDINATE& abs_c, const COORDINATE &rel_c, MATRIX& Cov)
{
	int ID_abs[4];
	RECEIVER_NODE* node_temp = new RECEIVER_NODE[4] ;
	//��Ϊ�������źſ������Բ�ͬ����
	//ID_abs������¼һ�鳬�����ľ���ID
	//node_temp������¼һ��ڵ�
	for (int i = 0; i < size; i ++) {
		ID_abs[i] = group[i] + ID[i];
		node_temp[i] = table.GetNode (ID_abs[i]);
	}
	switch (size) {
	case 1:
		package_single.SetCoordinate (abs_c);
		//���жϳ������ڵ��Ƿ�ı䣬����ı�����table
		if (!package_single.IsDifferent (ID_abs[0]))
			package_single.Update (rel_c, distance[0]);
		else {
			package_single.Reset (&node_temp[0]);
			package_single.Update (rel_c, distance[0]);
		}
		if (package_single.IsEnough ()) {
			// ֻ�е������㹻��ʱ��Ž��붨λ�㷨
			if(package_single.UltraLocalization () == 0) {
			//����ɹ��Ż��޸�init������GetCoordinate���ݸ�abs_c
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
	package_single.correction_count = CORRECTION_COUNT; // ˵��Euclidean Filter���ʱ����Է��������ˣ�
	package_multi.UltraLocalization ();
	abs_c = package_multi.GetCoordinate ();
	Cov.update (multiultra_error); //�����Ƕ೬�����Ķ�λ��������Ҫ�ǿ����������ľ��ȣ�����ʱ��һ���ȽϷ���
									//�Ժ���Ը�
	return 0;
}
