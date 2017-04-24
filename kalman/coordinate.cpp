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
	//尽管Node是COORDINATE的派生类，这在C++中是合法的
	//这里只会将派生类的基类部分给
	COORDINATE::operator = (Node);
	ID = Node.GetID ();
	return *this;
}
RECEIVER_NODE::~RECEIVER_NODE ( )
{
	//cout << "Receiver node destructor" << endl;
}
//这个是用于给RECEIVER_NODE初始赋值的时候用的，因为必须先申明一个RECEIVE_NODE数组
//而后循环依次给数组赋值，所以必须采用这种形式的Update
//当然，也可以自己重载一个“=”
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
//返回group所指定ID在Node表中的检索号
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
//复杂版的更新，更新RECEIVE_PACKAGE所有参数
void RECEIVE_PACKAGE::Update (const int Size, const RECEIVER_NODE* Node, const COORDINATE* Coordinate)
{
	//必须判断size是否改变，需要重新调整堆。
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

//简单版的更新，只更新Dist，认为当前接收到的超声波的ID顺序及数量都不变
void PACKAGE_MULTI::Update (const double* Dist, const int Size, 
							  const RECEIVER_NODE* Node, const COORDINATE* Coordinate)
{
	RECEIVE_PACKAGE::Update (Size, Node, Coordinate);
	for (int i = 0; i < size; i ++) {
		dist[i] = Dist[i];
	}
}
/*GetSub:当size=4时，通过GetSub得到size=3的子包来做平均计算。因为获得子
		包中存在某一个子包中的三个node共线导致无法计算，所以要剔除这样的
		子包，subnum的数值表示实际能够使用的子包。
input:		PACKAGE_MULTI**  subp		新生成的子包组数指针,这里subp在函数里面要重新给指针赋值，
										所以必须使用指针引用。
output:		int				subnum		实际能使用的子包数量，可能为零			*/

int  PACKAGE_MULTI::GetSub (PACKAGE_MULTI* subp)
{	
	double Dist[12];
    int j = 0;
    int subnum = 0;
	//开始全部初始化为true表示全都共线，后没测试一组不共线
    //子包数目+1
    bool mark[4] = { true, true, true, true };
	RECEIVER_NODE* Node = new RECEIVER_NODE[12];	
	/*选择模式	0:	1, 2, 3
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
/*TableUpdate:更新PARAMETER_PACKAGE中的position表，每次编码器获得新数据时，
  更新当前机器人局部坐标系的位置估计
	input:
		DELTA	d		两次超声波采样间编码器累积输入
		double	Dist	最新超声波测量				*/
void PACKAGE_SINGLE::Update (const COORDINATE rel_c, const double Dist)
{
	COORDINATE c;
	list_local_coordinate[front].Update (rel_c.GetX (), rel_c.GetY (), rel_c.GetZ ());
	list_dist[front] = Dist;
	//说明此时表第一次填满,将full标志位真
	if (full == false && ((front = (front + 1) % MAX_PACKAGE_SINGLE_TABLE_SIZE) == 0))
		full = true;
}
void PACKAGE_SINGLE::Reset (const RECEIVER_NODE* Node)
{
	front = 0;
	node[0] = *Node;
	full = false;
}
//这是用来判断当前机器人是否仍然处于原超声波传感器的覆盖范围内，使用绝对ID进行比较
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
/*IsUseful:对于唯一矢量和机器人与超声波节点连线的夹角小于20度的值予以剔除
		   使用余弦定理判别。选取的两组点分别为当前时刻的局部坐标系中的估计点和
		   倒退INCREMENT的局部坐标系估计点
Input:	输入为空，直接调用表中的front与front - INCREMENT表项所得获得的坐标增量
Output: true or false
*/
bool PACKAGE_SINGLE::IsUseful (int low, int high)
{
	double angle_tan_1, angle_tan_2;
	double x_local[2], y_local[2], dx, dy, x_global, y_global;
	//x_local,y_local表示在局部坐标系中两个时刻的坐标位置
	//从index小到大依次对应最近的数据->过去的数据
	x_local[0] = list_local_coordinate[high].GetX ();
	y_local[0] = list_local_coordinate[high].GetY ();
	x_local[1] = list_local_coordinate[low].GetX ();
	y_local[1] = list_local_coordinate[low].GetY ();
	//dx，dy表示位移矢量，sqrt(dx^2 + dy^2)表示对边长度
	dx = x_local[0] - x_local[1];
	dy = y_local[0] - y_local[1];
	//x_global,y_global表示在全局坐标系中估计的位置
	//从index小到大依次对应最近的数据->过去的数据
	x_global = init.GetX ();
	y_global = init.GetY ();
	// 比较全局坐标的tan和delta的tan
	angle_tan_1 = dy / dx;
	angle_tan_2 = y_global / x_global;
	//如果tan函数值比规定tan函数值要大，说明实际角度更小，返回false，不符合要求
	return (abs (angle_tan_2 - angle_tan_1) < tan (PI / 9)) ? false : true;
}

int	PACKAGE_SINGLE::GetTop () const
{
	return (front == 0)? MAX_PACKAGE_SINGLE_TABLE_SIZE - 1 : front - 1;
}

/* GetSub: 从表中提取出两组数据作差得到位移矢量，配合两组数据的超声波距离，
			获得一组size=2的等价PACKAGE_MULTI的子包。子包的距离矢量为[dist[high], dist[low]]
			节点矢量为[node[0], node[0] + delta]， delta为当前局部坐标估计减去前一时刻
			局部坐标估计。node[0]为PACKAGE_SINGLE中的收到信号的超声波节点位置。
    input:  void
	output: PACKAGE_MULTI&*/
 int PACKAGE_SINGLE::GetSub (PACKAGE_MULTI& subpackage)
{
	double delta[2], dist_temp[2], x_temp, y_temp, z_temp;
	RECEIVER_NODE node_temp[2];
	//只有使用指针才可以调用COORDINATE基类中的Update函数
	COORDINATE* node_ptr[2] = {&node_temp[0], &node_temp[1]};
	int low  = Decrement(INCREMENT),
		high = GetTop (); // 因为front总是指向下一个将要被填满的表项
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
