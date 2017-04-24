#include "math.h"
#include "stdlib.h"
#ifndef MATRIX_H
#define MATRIX_H

class MATRIX {
public:
	~MATRIX () { delete [] head; };
	MATRIX () {};
    MATRIX (int l, int w) ;
	MATRIX (const MATRIX &m);	
	MATRIX (const double* Vector, int length, int width);
	MATRIX& operator = (const MATRIX&);
	void	update ( MATRIX &m );
	void	multiple ( const MATRIX &m, MATRIX &result );
    void	add ( const MATRIX &m, MATRIX &result );
    void	sub ( const MATRIX &m, MATRIX &result );
	MATRIX&	trsp () const;
    void	inv ( MATRIX &result );
	int		GetLength () const { return length; };
	int		GetWidth () const { return width; };
	double* GetHead () const { return head; };
private:	
	int		length;
	int		width;
	double* head;
};

#endif
