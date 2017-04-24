#include "coordinate.h"
using namespace std;
COORDINATE::COORDINATE ()
{
	x = 0;
	y = 0;
	z = 0;
}
COORDINATE::COORDINATE (double X, double Y, double Z)
{
	x = X;
	y = Y;
	z = Z;
}
COORDINATE::COORDINATE (const COORDINATE &c)
{
	x = c.x;
	y = c.y;
	z = c.z;
}
COORDINATE& COORDINATE::operator = (const COORDINATE& Coordinate)
{
	x = Coordinate.x;
	y = Coordinate.y;
	z = Coordinate.z;
	return *this;
}
const COORDINATE& COORDINATE::operator - (const COORDINATE& Coordinate)
{
	double temp_x = x - Coordinate.x;
	double temp_y = y - Coordinate.y;
	double temp_z = z - Coordinate.z;
	return COORDINATE (temp_x, temp_y, temp_z);
}
void COORDINATE::Update (double X, double Y, double Z)
{
	x = X;
	y = Y;
	z = Z;
}
void COORDINATE::Update (const COORDINATE &c)
 {
	 x = c.x;
	 y = c.y;
	 z = c.z;
 }
double COORDINATE::GetX() const
{
	return x;
}
double COORDINATE::GetY() const
{
	return y;
}
double COORDINATE::GetZ() const
{
	return z;
}
bool COORDINATE::Collinear (const COORDINATE c1, const COORDINATE c2) const
{
	double temp[6];
	bool key[2];
	temp[0] = c2.x - c1.x;
	temp[1] = c1.x - x;
	temp[2] = c2.y - c1.y;
	temp[3] = c1.y - y;
	key[0] = ( temp[0] == temp[1] );
	key[1] = ( temp[2] == temp[3] );
    return ( key[0] && key[1] );
}

INPUT::INPUT ()
{
	translation[0] = 0;
	translation[1] = 0;
}
void INPUT::Reload (double left_inc, double right_inc) 
{
	translation[0] = left_inc;
	translation[1] = right_inc;	
	delta_angle = (translation[0] - translation[1])  / WHEEL_DISTANCE;
	delta_trans = (translation[0] + translation[1]) / 2;
}

double INPUT::GetAngle () const 
{
	return delta_angle;
}

double INPUT::GetTranslation () const 
{
	return delta_trans;
}
RECEIVER_NODE::RECEIVER_NODE()
{
	//cout << "Receiver node contructor" << endl;
}
RECEIVER_NODE::RECEIVER_NODE (const RECEIVER_NODE& r)
	: COORDINATE(r.x, r.y, r.z)
{
}
RECEIVER_NODE& RECEIVER_NODE::operator = (const RECEIVER_NODE& Node)
{
	//����Node��COORDINATE�������࣬����C++���ǺϷ���
	//����ֻ�Ὣ������Ļ��ಿ�ָ�
	COORDINATE::operator = (Node);
	ID = Node.GetID ();
	return *this;
}
RECEIVER_NODE::~RECEIVER_NODE ( )
{
	//cout << "Receiver node destructor" << endl;
}
//��������ڸ�RECEIVER_NODE��ʼ��ֵ��ʱ���õģ���Ϊ����������һ��RECEIVE_NODE����
//����ѭ�����θ����鸳ֵ�����Ա������������ʽ��Update
//��Ȼ��Ҳ�����Լ�����һ����=��
void RECEIVER_NODE::Update (COORDINATE c, int ID, double ks )
{
	COORDINATE::Update (c);
	RECEIVER_NODE::ID = ID;
	RECEIVER_NODE::ks = ks;
}
COORDINATE RECEIVER_NODE::GetCoordinate () const
{
	return COORDINATE(x, y, z);
}
int RECEIVER_NODE::GetID ( ) const
{
	return ID;
}

RECEIVER_TABLE::RECEIVER_TABLE ()
{
	COORDINATE temp;
	ifstream input;
	string inputstr;
	int ID;
	double ks, x, y, z;
    input.open ("TABLE.txt");
	do {
		input >> inputstr;
	} while ( inputstr != "$" );
	for ( int i = 0; i < MAX_RECEIVER_TABLE_SIZE; i ++ ) {
		input >> ID >> x >> y >> z >> ks;
		temp.Update ( x, y, z );
		node[i].Update ( temp, ID, ks );
	}
	node_count = MAX_RECEIVER_TABLE_SIZE;
}
RECEIVER_TABLE::~RECEIVER_TABLE () 
{
} 
//����group��ָ��ID��Node���еļ�����
int RECEIVER_TABLE::Search (int key) const
{
	int i = 0;
	do {
		if (node[i].GetID () == key ) return i;
		i ++;
	} while ( i < MAX_RECEIVER_TABLE_SIZE );
	return -1;
}
RECEIVER_NODE& RECEIVER_TABLE::GetNode (const int Iterator)
{
	//if (Iterator >= node_count || Iterator < 0)
		//cout << "Wrong Iterator! Exceed the limit!" << endl;
	return node[Iterator];
}
RECEIVE_PACKAGE::RECEIVE_PACKAGE ()
{
	size = 0;	
	node = new RECEIVER_NODE[4];
	//cout << "called" << endl;
}
RECEIVE_PACKAGE::RECEIVE_PACKAGE (const RECEIVE_PACKAGE& Package)
	: init (Package.GetCoordinate())
{
	size = Package.size;
	node = new RECEIVER_NODE[4];
	for (int i = 0; i < size; i ++)
		node[i] = Package.node[i];
}

bool RECEIVE_PACKAGE::EuclideanFilter (COORDINATE c) const
{
	double Euclidean = sqrt ((c.GetX () - init.GetX ()) * (c.GetX () - init.GetX ())
							+ (c.GetY () - init.GetY ()) * (c.GetY () - init.GetY ()));
	return (Euclidean > EUCLIDEAN_THRESHOLD) ? false : true;
}
//���Ӱ�ĸ��£�����RECEIVE_PACKAGE���в���
void RECEIVE_PACKAGE::Update (const int Size, const RECEIVER_NODE* Node, const COORDINATE* Coordinate)
{
	//�����ж�size�Ƿ�ı䣬��Ҫ���µ����ѡ�
	/*if (size != Size) {
		delete [] node;
		node = new RECEIVER_NODE[size];
		
	}*/
	size = Size;
	init = *Coordinate;
	for (int i = 0; i < size; i ++) {
		node[i] = Node[i];
	}
}


PACKAGE_MULTI::PACKAGE_MULTI() 
{
	dist = new double[4];
}
PACKAGE_MULTI::PACKAGE_MULTI (const PACKAGE_MULTI& Package) 
	:RECEIVE_PACKAGE(Package)
{
	dist = new double[4];
	for ( int i = 0; i < size; i ++ ) {
		dist[i] = Package.dist[i];
	}
}
PACKAGE_MULTI::~PACKAGE_MULTI ()
{
	delete []dist;
}

//�򵥰�ĸ��£�ֻ����Dist����Ϊ��ǰ���յ��ĳ�������ID˳������������
void PACKAGE_MULTI::Update (const double* Dist, const int Size, 
							  const RECEIVER_NODE* Node, const COORDINATE* Coordinate)
{
	RECEIVE_PACKAGE::Update (Size, Node, Coordinate);
	for (int i = 0; i < size; i ++) {
		dist[i] = Dist[i];
	}
}
/*GetSub:��size=4ʱ��ͨ��GetSub�õ�size=3���Ӱ�����ƽ�����㡣��Ϊ�����
		���д���ĳһ���Ӱ��е�����node���ߵ����޷����㣬����Ҫ�޳�������
		�Ӱ���subnum����ֵ��ʾʵ���ܹ�ʹ�õ��Ӱ���
input:		PACKAGE_MULTI**  subp		�����ɵ��Ӱ�����ָ��,����subp�ں�������Ҫ���¸�ָ�븳ֵ��
										���Ա���ʹ��ָ�����á�
output:		int				subnum		ʵ����ʹ�õ��Ӱ�����������Ϊ��			*/

int  PACKAGE_MULTI::GetSub (PACKAGE_MULTI* subp)
{	
	double Dist[12];
    int j = 0;
    int subnum = 0;
	//��ʼȫ����ʼ��Ϊtrue��ʾȫ�����ߣ���û����һ�鲻����
    //�Ӱ���Ŀ+1
    bool mark[4] = { true, true, true, true };
	RECEIVER_NODE* Node = new RECEIVER_NODE[12];	
	/*ѡ��ģʽ	0:	1, 2, 3
				1:	2, 3, 0
				2:	3, 1, 0
				3:	0, 1, 2					*/
	for (int i = 0; i < 4; i ++) {
		for (int j = 0; j < 3; j ++) {
            Dist[i * 3 + j] = dist[(i + j + 1) % size];
			Node[i * 3 + j] = node[(i + j + 1) % size];
		}
		if (!Node[i * 3].Collinear (Node[i * 3 + 1].GetCoordinate (), 
									Node[i * 3 + 2].GetCoordinate ())) {
			subnum ++;
			mark[i] = false;
		}
	}
    if (subnum != 0) {
		for ( int i = 0; i < 4; i ++ ) {
			if (! mark[i]) {
                subp[j].Update (&Dist[i * 3], 3, &Node[i * 3], &init);
                j ++;
			}
		}
        return subnum;
	}		
	else
        return 0;
}

PACKAGE_SINGLE::PACKAGE_SINGLE () : RECEIVE_PACKAGE ()
{
	list_local_coordinate   = new COORDINATE[MAX_PACKAGE_SINGLE_TABLE_SIZE];
	list_dist			    = new double[MAX_PACKAGE_SINGLE_TABLE_SIZE];
	full					= false;	
	front					= 0;
	correction_count		= 0;
}

PACKAGE_SINGLE::~PACKAGE_SINGLE ()
{
	delete []list_local_coordinate;
	delete []list_dist;
}
bool PACKAGE_SINGLE::IsEnough ()
{
	return (full == 1 || front > INCREMENT) ? true : false;
}
/*TableUpdate:����PARAMETER_PACKAGE�е�position��ÿ�α��������������ʱ��
  ���µ�ǰ�����˾ֲ�����ϵ��λ�ù���
	input:
		DELTA	d		���γ�����������������ۻ�����
		double	Dist	���³���������				*/
void PACKAGE_SINGLE::Update (const COORDINATE rel_c, const double Dist)
{
	COORDINATE c;
	list_local_coordinate[front].Update (rel_c.GetX (), rel_c.GetY (), rel_c.GetZ ());
	list_dist[front] = Dist;
	//˵����ʱ���һ������,��full��־λ��
	if (full == false && ((front = (front + 1) % MAX_PACKAGE_SINGLE_TABLE_SIZE) == 0))
		full = true;
}
void PACKAGE_SINGLE::Reset (const RECEIVER_NODE* Node)
{
	front = 0;
	node[0] = *Node;
	full = false;
}
//���������жϵ�ǰ�������Ƿ���Ȼ����ԭ�������������ĸ��Ƿ�Χ�ڣ�ʹ�þ���ID���бȽ�
bool PACKAGE_SINGLE::IsDifferent (const int ID)
{
	return (node[0].GetID () == ID) ? false : true;  
}
int	PACKAGE_SINGLE::Decrement (int interval)
{
	if (front - interval < 0)
		return MAX_PACKAGE_SINGLE_TABLE_SIZE - (interval - front) + 1;
	else
		return front - interval;

}
MATRIX PACKAGE_SINGLE::UncertaintyEvaluation ()
{
	//    A = [ r(1) - delta(1), r(2) - delta(2) ;
      //       r(1),            r(2)];	
	//     factor = 2 * [ dist(1),    0,       (r(1) - delta(1)), (r(2) - delta(2));
        //           0,          dist(2), 0,                 0   ];
	//    H = A ^ (-1) * factor;
    // y = H * error * H';
	double delta[2], dist[2];
	int low  = Decrement(INCREMENT),
		high = GetTop ();
	delta[0] = list_local_coordinate[high].GetX () - list_local_coordinate[low].GetX ();
	delta[1] = list_local_coordinate[high].GetY () - list_local_coordinate[low].GetY ();
	dist[0] = list_dist[high];
	dist[1] = list_dist[low];
	const double vector_A[4] = {init.GetX () - delta[0], init.GetY () - delta[1],
								init.GetX (),			  init.GetY ()};
	MATRIX A(vector_A, 2, 2);
	const double vector_factor[8] = {dist[0],	0,	init.GetX () - dist[0], init.GetY () - dist[1],
									 0,		dist[1],0,						0};
	MATRIX factor (vector_factor, 2, 4);
	MATRIX H (A.GetLength (), factor.GetWidth ());
	MATRIX y (H.GetLength (), H.GetLength ());
	MATRIX temp (H.GetLength (), error.GetWidth ());
	A.inv (A); // A ^ (-1)
	A.multiple (factor, H); // A ^ (-1) * factor
	H.multiple (error, temp); //H * error
	temp.multiple (H.trsp (), y); //  y = H * error * H'
	return y;
}
/*IsUseful:����Ψһʸ���ͻ������볬�����ڵ����ߵļн�С��20�ȵ�ֵ�����޳�
		   ʹ�����Ҷ����б�ѡȡ�������ֱ�Ϊ��ǰʱ�̵ľֲ�����ϵ�еĹ��Ƶ��
		   ����INCREMENT�ľֲ�����ϵ���Ƶ�
Input:	����Ϊ�գ�ֱ�ӵ��ñ��е�front��front - INCREMENT�������û�õ���������
Output: true or false
*/
bool PACKAGE_SINGLE::IsUseful (int low, int high)
{
	double angle_tan_1, angle_tan_2;
	double x_local[2], y_local[2], dx, dy, x_global, y_global;
	//x_local,y_local��ʾ�ھֲ�����ϵ������ʱ�̵�����λ��
	//��indexС�������ζ�Ӧ���������->��ȥ������
	x_local[0] = list_local_coordinate[high].GetX ();
	y_local[0] = list_local_coordinate[high].GetY ();
	x_local[1] = list_local_coordinate[low].GetX ();
	y_local[1] = list_local_coordinate[low].GetY ();
	//dx��dy��ʾλ��ʸ����sqrt(dx^2 + dy^2)��ʾ�Ա߳���
	dx = x_local[0] - x_local[1];
	dy = y_local[0] - y_local[1];
	//x_global,y_global��ʾ��ȫ������ϵ�й��Ƶ�λ��
	//��indexС�������ζ�Ӧ���������->��ȥ������
	x_global = init.GetX ();
	y_global = init.GetY ();
	// �Ƚ�ȫ�������tan��delta��tan
	angle_tan_1 = dy / dx;
	angle_tan_2 = y_global / x_global;
	//���tan����ֵ�ȹ涨tan����ֵҪ��˵��ʵ�ʽǶȸ�С������false��������Ҫ��
	return (abs (angle_tan_2 - angle_tan_1) < tan (PI / 9)) ? false : true;
}

int	PACKAGE_SINGLE::GetTop () const
{
	return (front == 0)? MAX_PACKAGE_SINGLE_TABLE_SIZE - 1 : front - 1;
}

/* GetSub: �ӱ�����ȡ��������������õ�λ��ʸ��������������ݵĳ��������룬
			���һ��size=2�ĵȼ�PACKAGE_MULTI���Ӱ����Ӱ��ľ���ʸ��Ϊ[dist[high], dist[low]]
			�ڵ�ʸ��Ϊ[node[0], node[0] + delta]�� deltaΪ��ǰ�ֲ�������Ƽ�ȥǰһʱ��
			�ֲ�������ơ�node[0]ΪPACKAGE_SINGLE�е��յ��źŵĳ������ڵ�λ�á�
    input:  void
	output: PACKAGE_MULTI&*/
 int PACKAGE_SINGLE::GetSub (PACKAGE_MULTI& subpackage)
{
	double delta[2], dist_temp[2], x_temp, y_temp, z_temp;
	RECEIVER_NODE node_temp[2];
	//ֻ��ʹ��ָ��ſ��Ե���COORDINATE�����е�Update����
	COORDINATE* node_ptr[2] = {&node_temp[0], &node_temp[1]};
	int low  = Decrement(INCREMENT),
		high = GetTop (); // ��Ϊfront����ָ����һ����Ҫ�������ı���
	if (IsUseful (low,high)) {
		delta[0] = list_local_coordinate[high].GetX () - list_local_coordinate[low].GetX ();
		delta[1] = list_local_coordinate[high].GetY () - list_local_coordinate[low].GetY ();
		dist_temp[0] = list_dist[high];
		dist_temp[1] = list_dist[low];
		node_temp[0] = *node;
		x_temp = node->GetX () + delta[0];
		y_temp = node->GetY () + delta[1];
		z_temp = node->GetZ ();
		node_ptr[1]->Update (x_temp, y_temp, z_temp);
		subpackage.Update (dist_temp, 2, node_temp, &init);
		return 1;
	}
	else return 0;
}
