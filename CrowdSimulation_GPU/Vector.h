#ifndef _WZVECTOR_H
#define _WZVECTOR_H

#include <math.h>

class Vector2D {
private:
	double m_dX;
	double m_dY;

public:
	Vector2D();
	Vector2D(double x, double y);
    
    Vector2D normalize();
    
    Vector2D roundV();
    Vector2D floorV();
    Vector2D ceilingV();

	Vector2D& operator+= (const Vector2D& b);
	Vector2D& operator-= (const Vector2D& b);

	Vector2D& operator/= (const double scale);
	Vector2D& operator*= (const double scale);

	double dot(const Vector2D& b);

	double getX()const{return m_dX;}
	void setX(double x){m_dX = x;}

	double getY()const{return m_dY;}
	void setY(double y){m_dY = y;}

	friend Vector2D operator+ (const Vector2D& a, const Vector2D& b);
	friend Vector2D operator- (const Vector2D& a, const Vector2D& b);
	friend Vector2D operator/ (const Vector2D& a, const double scale);
	friend Vector2D operator* (const Vector2D& a, const double scale);
    friend Vector2D operator* (const double scale, const Vector2D& a);
	
	double length()const{return m_dX*m_dX + m_dY*m_dY;}
	double squaredLength()const{return sqrt(this->length());}
    double distanceTo(const Vector2D& vec);

	static Vector2D vec2Zero;
};

class Vector3D {
private:
	double m_dX;
	double m_dY;
	double m_dZ;

public:
	Vector3D();
	Vector3D(double x, double y, double z);

	Vector3D& operator+= (const Vector3D& b);
	Vector3D& operator-= (const Vector3D& b);
	Vector3D& operator/= (const double scale);
	Vector3D& operator*= (const double scale);

	double dot(const Vector3D& b);

	friend Vector3D operator+ (const Vector3D& a, const Vector3D& b);
	friend Vector3D operator- (const Vector3D& a, const Vector3D& b);
	friend Vector3D operator/ (const Vector3D& a, const double scale);
    friend Vector3D operator* (const Vector3D& a, const double scale);
    friend Vector3D operator* (const double scale, const Vector3D& a);

	double length()const{return m_dX*m_dX + m_dY*m_dY + m_dZ*m_dZ;}
	double squaredLength()const{return sqrt(this->length());}

	static Vector3D vec3Zero;
};

#endif