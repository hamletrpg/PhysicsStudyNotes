#include "vectors.h"
#include <cmath>
#include <cfloat>

// macro for epsilon style comparison, due to floating point error
#define CMP(x, y)                       \
    (fabsf((x) - (y)) <= FLT_EPSILON *  \
    fmax(1.0f,                          \
    fmaxf(fabsf(x), fabsf(y))))                                                 

// Sum operator overloading
vec2 operator+(const vec2& l, const vec2& r)
{
    return {l.x + r.x, l.y + r.y};
}
vec3 operator+(const vec3& l, const vec3& r)
{
    return {l.x + r.x, l.y + r.y, l.z + r.z};
}

// Substraction operator overloading
vec2 operator-(const vec2& l, const vec2& r)
{
    return {l.x - r.x, l.y - r.y};
}
vec3 operator-(const vec3& l, const vec3& r)
{
    return {l.x - r.x, l.y - r.y, l.z - r.z};
}
// Multiplication operator overloading
vec2 operator*(const vec2& l, const vec2& r)
{
    return {l.x * r.x, l.y * r.y};
}
vec3 operator*(const vec3& l, const vec3& r)
{
    return {l.x * r.x, l.y * r.y, l.z * r.z};
}

// Scalar multiplication
vec2 operator*(const vec2& l, float r)
{
    return {l.x * r, l.y * r};
}
vec3 operator*(const vec3& l, float r)
{
    return {l.x * r, l.y * r, l.z * r};
}

// Vector equality operations
bool operator==(const vec2& l, const vec2& r)
{
    return CMP(l.x, r.x) && CMP(l.y, r.y);
}
bool operator==(const vec3& l, const vec3& r)
{
    return CMP(l.x, r.x) && CMP(l.y, r.y) && CMP(l.z, r.z);
}

// Vector inequality operations
bool operator!=(const vec2& l, const vec2& r)
{
    return !(l == r);
}
bool operator!=(const vec3& l, const vec3& r)
{
    return !(l == r);
}

// Dot Product
float Dot(const vec2& l, const vec2& r)
{
    return l.x * r.x + l.y * r.x;
}
float Dot(const vec3& l, const vec3& r)
{
    return l.x * r.x + l.y * r.y + l.z * r.z;
}

// Magnitude
float Magnitude(const vec2& v)
{
    return sqrtf(Dot(v, v));
}

float Magnitude(const vec3& v)
{
    return sqrtf(Dot(v, v));
}

float MagnitudeSq(const vec2& v)
{
    return Dot(v, v);
}
float MagnitudeSq(const vec3& v)
{
    return Dot(v, v);
}

// Normalized Vectors
void Normalize(vec2& v)
{
    v = v *(1.0f / Magnitude(v));
}
void Normalize(vec3& v)
{
    v = v * (1.0f / Magnitude(v));
}

vec2 Normalized(const vec2& v)
{
    return v * (1.0f / Magnitude(v));
}
vec3 Normalized(const vec3& v)
{
    return v * (1.0f / Magnitude(v));
}

// Cross Product
vec3 Cross(const vec3& l, const vec3& r)
{
    vec3 result;
    result.x = l.y * r.z - l.z * r.y;
    result.y = l.z * r.x - l.x * r.z;
    result.z = l.x * r.y - l.y * r.x;
    return result;
}

// Angle
float Angle(const vec2& l, const vec2& r)
{
    float m = sqrt(MagnitudeSq(l) * Magnitude(r));
    return acos(Dot(l, r) / m);
}
float Angle(const vec3& l, const vec3& r)
{
    float m = sqrtf(MagnitudeSq(l) * MagnitudeSq(r));
    return acos(Dot(l, r) / m);
}

// Projection
vec2 Project(const vec2& length, const vec2& direction)
{
    float dot = Dot(length, direction);
    float magSq = MagnitudeSq(direction);
    return direction * (dot/magSq);
}
vec3 Project(const vec3& length, const vec3& direction)
{
    float dot = Dot(length, direction);
    float magSq = MagnitudeSq(direction);
    return direction * (dot/ magSq);
}

vec2 Perpensicular(const vec2& len, const vec2& dir)
{
    return len - Project(len, dir);
}
vec3 Perpendicular(const vec3& len, const vec3& dir)
{
    return len - Project(len, dir);
}

// Reflection
vec2 Reflection(const vec2& vec, const vec2& normal)
{
    float d = Dot(vec, normal);
    return vec - normal * (d * 2.0f);
}