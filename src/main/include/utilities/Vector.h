#pragma once
#include "math.h"

class Vector
{
public:
    float x, y; // coordinates

    /**
     * constructor for 2D vector
     * given x and y coordinates
     * */
    Vector(float x = 0, float y = 0)
    {
        this->x = x;
        this->y = y;
    }

    // convert from degrees to radians
    float radians(float const &degrees)
    {
        return degrees * M_PI / 180;
    }

    // convert from radians to degrees
    float degrees(float const &radians)
    {
        return radians * 180 / M_PI;
    }

    // return this vector's magnitude as a float
    float getMagnitude()
    {
        return hypot(x, y);
    }

    // add another vector to this vector and return the result
    void add(Vector const &obj)
    {
        x += obj.x;
        y += obj.y;
    }

    Vector getAdded(Vector const &obj)
    {

        return Vector{x + obj.x, y + obj.y};
    }

    // subtract another vector from this vector
    void subtract(Vector const &obj)
    {
        x -= obj.x;
        y -= obj.y;
    }

    Vector getSubtracted(Vector &obj)
    {

        return Vector{x - obj.x, y - obj.y};
    }

    // return this vector after scaling by the given constant
    void scale(float const &k)
    {
        x *= k;
        y *= k;
    }

    // scale this vector by a constant
    Vector getScaled(float const &k)
    {
        return Vector{x * k, y * k};
    }

    // return this vector after dividing by the given constant
    void divide(float const &k)
    {
        x /= k;
        y /= k;
    }

    Vector getDivided(float const &k)
    {
        return Vector{x / k, y / k};
    }

    // return this vector's angle (-180 to 180 degrees)
    float getAngle()
    {
        return degrees(atan2(x, y));
    }

    // rotate this vector clockwise by the given angle
    void rotateCW(float angle)
    {
        angle = radians(angle);
        *this = Vector{x * cos(angle) + y * sin(angle), y * cos(angle) - x * sin(angle)};
    }

    // return this vector rotated by the given angle
    Vector getRotatedCW(float angle)
    {
        angle = radians(angle);
        return Vector{x * cos(angle) + y * sin(angle), y * cos(angle) - x * sin(angle)};
    }

    // reset the vector to {0, 0}
    void reset()
    {
        x = 0;
        y = 0;
    }

    // float getMagnitudeOfProjectionOnto(Vector &baseVector)
    // {
    //     return cos(radians(this->getAngle() - baseVector.getAngle())) * getMagnitude();
    // }
};

/*
includes the abs() function for Vectors
*/
namespace t2D {
    // return the magnitude of a Vector
    float abs(Vector const &obj) {
        return std::hypot(obj.x, obj.y);
    }
}
