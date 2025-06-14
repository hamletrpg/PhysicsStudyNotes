#ifndef _H_MATH_VECTORS_
#define _H_MATH_VECTORS

// the C standard librady acos return values in radians rather than degree
// It's more intuitive to work with degrees when working with angles
// so these macros will come handy
#define RAD2DEG(x) ((x) * 57.29574f)
#define DEG2RAD(x) ((x) * 0.0174533f)

typedef struct vec2 
{
    union
    {
        struct 
        {
            float x;
            float y;
        };
        float asArray[2];
    };
    float& operator[](int i)
    {
        return asArray[i];
    }
} vec2;

typedef struct vec3
{
    union 
    {
        struct
        {
            float x;
            float y;
            float z;
        };
        float asArray[3];
    };
    float& operator[](int i)
    {
        return asArray[i];
    }
} vec3;

vec2 operator+(const vec2& l, const vec2& r);
vec3 operator+(const vec3& l, const vec3& r);
vec2 operator-(const vec2& l, const vec2& r);
vec3 operator-(const vec3& l, const vec3& r);
vec2 operator*(const vec2& l, const vec2& r);
vec3 operator*(const vec3& l, const vec3& r);
vec2 operator*(const vec2& l, float r);
vec3 operator*(const vec3& l, float r);
bool operator==(const vec2& l, const vec2& r);
bool operator==(const vec3& l, const vec3& r);
bool operator!=(const vec2& l, const vec2& r);
bool operator!=(const vec3& l, const vec3& r);
float Dot(const vec2& l, const vec2& r);
float Dot(const vec3& l, const vec3& r);
// Some magic is being done to avoid expensive operations regarding checking for square root
// Rather we apply the dot product with the vector we want to get the magnitude with itself
// Normally we use the magnitude/lenght of a vector to compare it with known numbers (make sense)
// Taking square root for comparation is expensive for such a "trivial task"
// ex:
// if (Magnitude(someVector) < 5.0) { do or don't do something }
// we'd rather do
// if (MagnitudeSq(someVector)) < 5.0f * 5.0f {do the thing}
float Magnitude(const vec2& v);
float Magnitude(const vec3& v);
float MagnitudeSq(const vec2& v);
float MagnitudeSq(const vec3& v);
void Normalize(vec2& v);
void Normalize(vec3& v);
vec2 Normalized(const vec2& v);
vec3 Normalized(const vec3& v);
vec3 Cross(const vec3& l, const vec3& r);
float Angle(const vec2& l, const vec2& r);
float Angle(const vec3& l, const vec3& r);
vec2 Project(const vec2& length, const vec2& direction);
vec3 Project(const vec3& length, const vec3& direction);
vec2 Perpendicular(const vec2& len, const vec2& dir);
vec3 Perpendicular(const vec3& len, const vec3& dir);
vec2 Reflection(const vec2& vec, const vec2& normal);
vec3 Reflection(const vec3& vec, const vec3& normal);

#endif