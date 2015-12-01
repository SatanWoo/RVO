#include "Vector.h"

Vector2D Vector2D::vec2Zero = Vector2D(0, 0);

Vector2D::Vector2D():m_dX(0.0), m_dY(0.0)
{}

Vector2D::Vector2D(double x, double y):m_dX(x), m_dY(y)
{}

Vector2D Vector2D::normalize()
{
    double length = squaredLength();
    return Vector2D(m_dX/length, m_dY/length);
}

Vector2D& Vector2D::operator+= (const Vector2D& b)
{
	this->m_dX += b.m_dX;
	this->m_dY += b.m_dY;

	return *this;
}

Vector2D& Vector2D::operator-= (const Vector2D& b)
{
	this->m_dX -= b.m_dX;
	this->m_dY -= b.m_dY;

	return *this;
}

Vector2D& Vector2D::operator*= (const double scale)
{
	this->m_dX *= scale;
	this->m_dY *= scale;

	return *this;
}

Vector2D& Vector2D::operator/= (const double scale)
{
	this->m_dX /= scale;
	this->m_dY /= scale;

	return *this;
}

double Vector2D::dot(const Vector2D& b)
{
	return this->m_dX * b.m_dX + this->m_dY * m_dY;
}

// Friend
Vector2D operator+ (const Vector2D& a, const Vector2D& b)
{
	return Vector2D(a.m_dX + b.m_dX, a.m_dY + b.m_dY);
}

Vector2D operator- (const Vector2D& a, const Vector2D& b)
{
	return Vector2D(a.m_dX - b.m_dX, a.m_dY - b.m_dY);
}

Vector2D operator/ (const Vector2D& a, const double scale)
{
	return Vector2D(a.m_dX / scale, a.m_dY / scale);
}

Vector2D operator* (const Vector2D& a, const double scale)
{
	return Vector2D(a.m_dX * scale, a.m_dY * scale);
}

Vector2D operator* (const double scale, const Vector2D& a)
{
    return Vector2D(a.m_dX * scale, a.m_dY * scale);
}

double Vector2D::distanceTo(const Vector2D &vec)
{
    double diffX = vec.getX() - m_dX;
    double diffY = vec.getY() - m_dY;
    
    return sqrt(diffX * diffX + diffY * diffY);
}


Vector2D Vector2D::roundV()
{
    return Vector2D(double((int)(m_dX)), double((int)(m_dY)));
}

Vector2D Vector2D::floorV()
{
    return Vector2D(floor(m_dX), floor(m_dY));
}

Vector2D Vector2D::ceilingV()
{
    return Vector2D(ceil(m_dX), ceil(m_dY));
}

// Vector 3D
Vector3D Vector3D::vec3Zero = Vector3D(0, 0, 0);

Vector3D::Vector3D():m_dX(0.0), m_dY(0.0), m_dZ(0.0)
{}

Vector3D::Vector3D(double x, double y, double z):m_dX(x),m_dY(y),m_dZ(z)
{}

Vector3D& Vector3D::operator+=(const Vector3D& b)
{
	this->m_dX += b.m_dX;
	this->m_dY += b.m_dY;
	this->m_dZ += b.m_dZ;

	return *this;
}

Vector3D& Vector3D::operator-=(const Vector3D& b)
{
	this->m_dX -= b.m_dX;
	this->m_dY -= b.m_dY;
	this->m_dZ -= b.m_dZ;

	return *this;
}

Vector3D& Vector3D::operator*=(const double scale)
{
	this->m_dX *= scale;
	this->m_dY *= scale;
	this->m_dZ *= scale;

	return *this;
}

Vector3D& Vector3D::operator/=(const double scale)
{
	this->m_dX /= scale;
	this->m_dY /= scale;
	this->m_dZ /= scale;

	return *this;
}

double Vector3D::dot(const Vector3D& b)
{
	return this->m_dX * b.m_dX + this->m_dY * b.m_dY + this->m_dZ * b.m_dZ;
}

Vector3D operator+(const Vector3D& a, const Vector3D& b)
{
	return Vector3D(a.m_dX + b.m_dX, a.m_dY + b.m_dY, a.m_dZ + b.m_dZ);
}

Vector3D operator-(const Vector3D& a, const Vector3D& b)
{
	return Vector3D(a.m_dX - b.m_dX, a.m_dY - b.m_dY, a.m_dZ - b.m_dZ);
}

Vector3D operator*(const Vector3D& a, const double scale)
{
	return Vector3D(a.m_dX * scale, a.m_dY * scale, a.m_dZ * scale);
}

Vector3D operator*(const double scale, const Vector3D& a)
{
    return Vector3D(a.m_dX * scale, a.m_dY * scale, a.m_dZ * scale);
}

Vector3D operator/(const Vector3D& a, const double scale)
{
	return Vector3D(a.m_dX / scale, a.m_dY / scale, a.m_dZ / scale);
}
