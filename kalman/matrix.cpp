#include "matrix.h"
#include "iostream"
using namespace std;


MATRIX::MATRIX ( const MATRIX &m )
{
	length = m.length;
	width  = m.width;
	head   = new double[m.length * m.width];
	for ( int i = 0; i < m.length * m.width; i ++ )	{
		head[i] = m.head[i];
	}
}

MATRIX::MATRIX ( int length, int width )
{
    MATRIX::length = length;
    MATRIX::width  = width;
    head   = new double[length * width];
}

MATRIX::MATRIX (const double* Vector, int length, int width)
{
	MATRIX::length = length;
	MATRIX::width = width;
	head   = new double[MATRIX::length * MATRIX::width];	
	for (int i = 0; i < length * width; i ++) {
		head[i] = Vector[i];
	}
}
MATRIX& MATRIX::operator = (const MATRIX& Matrix)
{
	length = Matrix.length;
	width = Matrix.width;
	for (int i = 0; i < length * width; i ++) 
		head[i] = Matrix.head[i];
	return *this;
}
void MATRIX::update ( MATRIX &m )
{
	for ( int i = 0; i < length * width; i ++ )
		head[i] = m.head[i];
}

void MATRIX::multiple ( const MATRIX& m, MATRIX &result )
{
    for ( int i = 0; i < length; i ++ ) {
        for( int j = 0;j < m.width;j ++)
        {
			result.head[m.width * i + j] = 0;
		  for( int k = 0; k < width; k ++ )
			  result.head[m.width * i + j] = result.head[m.width * i + j] + head[width * i + k] * m.head[m.width * k + j];
        }
	}
}

void MATRIX::add ( const MATRIX& m, MATRIX &result )
{
    for ( int i = 0; i < length; i ++ )
        for( int j = 0; j < width; j ++ )
			result.head[width * i + j] = head[width * i + j] + m.head[width * i + j];
}

void MATRIX::sub ( const MATRIX& m, MATRIX &result )
{
    for ( int i = 0; i < length; i ++ )
        for( int j = 0; j < width; j ++ )
			result.head[width * i + j] = head[width * i + j] - m.head[width * i + j];
}

MATRIX& MATRIX::trsp () const
{	
	MATRIX* result = new MATRIX(width, length);
    for ( int i = 0; i < length; i ++ ) {
        for( int j = 0; j < width; j ++ )
			result->head[j * length + i] = head[width * i + j];
	}
	return *result;
}

void MATRIX::inv ( MATRIX &result)
{
    int i, j, iPass, imx, icol, irow;
    double det, temp, pivot, factor;	
	MATRIX A ( *this );
    det = 1;
    for ( i = 0; i < length; i++ )//set result a unit matrix
    {
        for (j = 0; j < length; j++)
			result.head[length * i + j] = 0;
		result.head[length * i + i] = 1;
    }


    for ( iPass = 0; iPass < length; iPass++ )
    {
        imx = iPass;
        for ( irow = iPass; irow < length; irow ++ )
        {
            if (fabs(A.head[length * irow + iPass]) > fabs(A.head[length*imx+iPass])) imx = irow;
        }
        if (imx != iPass)
        {
            for (icol = 0; icol < length; icol++)
            {
                temp = result.head[length*iPass+icol];
                result.head[length*iPass+icol] = result.head[length*imx+icol];
                result.head[length*imx+icol] = temp;
                if (icol >= iPass)
                {
                    temp = A.head[length*iPass+icol];
                    A.head[length*iPass+icol] = A.head[length*imx+icol];
                    A.head[length*imx+icol] = temp;
                }
            }
        }

        pivot = A.head[length*iPass+iPass];
        det = det * pivot;

        for (icol = 0; icol < length; icol++)
        {

            result.head[length*iPass+icol] = result.head[length*iPass+icol] / pivot;
            if (icol >= iPass) A.head[length*iPass+icol] = A.head[length*iPass+icol] / pivot;
        }

        for (irow = 0; irow < length; irow++)
        {

            if (irow != iPass) factor = A.head[length*irow+iPass];
            for (icol = 0; icol < length; icol++)
            {
                if (irow != iPass)
                {
                    result.head[length*irow+icol] -= factor * result.head[length*iPass+icol];
                    A.head[length*irow+icol] -= factor * A.head[length*iPass+icol];
                }
            }
        }
    }
}