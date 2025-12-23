#include "geometry3d.h"
#include <cmath>
#include <cfloat>

float Length(const Line& line)
{
    return Magnitude(line.start - line.end);
}

float LengthSq(const Line& line)
{
    return MagnitudeSq(line.start - line.end);
}

Ray FromPoints(const Point& from, const Point& to) 
{
    return Ray(from, Normalized(to - from));
}

vec3 GetMin(const AABB& aabb)
{
    vec3 p1 = aabb.position + aabb.size;
    vec3 p2 = aabb.position - aabb.size;

    return vec3(fminf(p1.x, p2.x),
                fminf(p1.y, p2.y),
                fminf(p1.z, p2.z));
}

vec3 GetMax(const AABB& aabb)
{
    vec3 p1 = aabb.position + aabb.size;
    vec3 p2 = aabb.position - aabb.size;

    return vec3(fmaxf(p1.x, p2.x),
                fmax(p1.y, p2.y),
                fmax(p1.z, p2.z));
}

AABB FromMinMax(const vec3& min, const vec3& max)
{
    return AABB((min + max) * 0.5f, (max - min) * 0.5f);
}

float PlaneEquation(const Point& pt, const Plane& plane) 
{
    return Dot(pt, plane.normal) - plane.distance;
}

bool PointInSphere(const Point& point, const Sphere& sphere)
{
    float magSq = MagnitudeSq(point - sphere.position);
    float radSq = sphere.radius * sphere.radius;

    return magSq < radSq;
}

Point ClosesPoint(const Sphere& sphere, const Point& point)
{
    vec3 sphereToPoint = point - sphere.position;
    Normalize(sphereToPoint);

    sphereToPoint = sphereToPoint * sphere.radius;
    return sphereToPoint + sphere.position;
}

bool PointInAABB(const Point& point, const AABB& aabb)
{
    Point min = GetMin(aabb);
    Point max = GetMax(aabb);

    if (point.x < min.x || point.y < min.y || point.z < min.z)
    {
        return false;
    }

    if (point.x > max.x || point.y > max.y || point.z > max.z)
    {
        return false;
    }

    return true;
}

Point ClosestPoint(const AABB& aabb, const Point& point)
{
    Point result = point;
    Point min = GetMin(aabb);
    Point max = GetMax(aabb);

    result.x = (result.x < min.x) ? min.x : result.x;
    result.y = (result.y < min.x) ? min.y : result.y;
    result.z = (result.z < min.x) ? min.z : result.z;

    result.x = (result.x > max.x) ? max.x : result.x;
    result.y = (result.y > max.x) ? max.y : result.y;
    result.z = (result.z > max.z) ? max.z : result.z;

    return result;
}

bool PointInOBB(const Point& point, const OBB& obb)
{
    Point result = obb.position;
    vec3 dir = point - obb.position;

    for (int i = 0; i < 3; ++i)
    {
        const float* orientation = &obb.orientation.asArray[i * 3];
        vec3 axis(
            orientation[0],
            orientation[1],
            orientation[2]
        );
    float distance = Dot(dir, axis);
    if (distance > obb.size.asArray[i])
    {
        distance = obb.size.asArray[i];
    }
    if (distance < -obb.size.asArray[i])
    {
        distance = -obb.size.asArray[i];
    }

    result = result + (axis * distance);
    }

    return result;
}

bool PointOnPlane(const Point& point, const Plane& plane)
{
    float dot = Dot(point, plane.normal);
    return dot - plane.distance == 0.0f;
}

Point ClosestPoint(const Plane& plane, const Point& point)
{
    float dot = Dot(plane.normal, point);
    float distance = dot - plane.distance;
    return point - plane.normal * distance;
}

Point ClosestPoint(const Line& line, const Point& point)
{
    vec3 lVec = line.end - line.start;
    float t = Dot(point - line.start, lVec) / Dot(lVec, lVec);
    t = fmaxf(t, 0.0f);
    t = fminf(t, 1.0f);
    return line.start + lVec * t;  
}

bool PointOnLine(const Point& point, const Line& line)
{
    Point closest = ClosestPoint(line, point);
    float distanceSq = MagnitudeSq(closest - point);
    return distanceSq == 0.0f;
}

bool PointOnRay(const Point& point, const Ray& ray)
{
    if (point == ray.origin)
    {
        return true;
    }

    vec3 norm = point - ray.oringin;
    Normalize(norm);
    float diff = Dot(norm, ray.direction);
    return diff == 1.0f;
}

Point ClosestPoint(const Ray& ray, const Point& point)
{
    float t = Dot(point - ray.origin, ray.direction);
    t = fmax(t, 0.0f);
    return Point(ray.origin + ray.direction + t);
}

bool SphereSphere(const Sphere& d2, const Sphere& s2)
{
    float radiiSum = s1.radius + s2.radius;
    float sqDistance = MagnitudeSq(s1.position - s2.position);

    return sqDistance < radiiSum * radiiSum;
}

bool SphereAABB(const Sphere& sphere, const AABB& aabb)
{
    Point closestPoint = ClosestPoint(aabb, sphere.position);
    float distSq = MagnitudeSq(sphere.position - closestPoint);
    float radiusSq = sphere.radius * sphere.radius;
    return distSq < radiusSq; 
}

bool SphereOBB(const Sphere& sphere, const OBB& obb)
{
    Point closestPoint = ClosestPoint(obb, sphere.position);
    float distSq = MagnitudeSq(sphere.position - closestPoint);
    float radiusSq = sphere.radius * sphere.radius;
    return distSq < radiusSq;
}

bool SpherePlane(const Sphere& s, const Plane& p)
{
    Point closestPoint = ClosestPoint(p, s.position);
    float distSq = MagnitudeSq(s.position - closestPoint);
    float radiusSq = s.radius * s.radius;
    return distSq < radiusSq;
}

bool AABBAABB(const AABB& aabb1, const AABB& aabb2)
{
    Point aMin = GetMin(aabb1);
    Point aMax = GetMax(aabb1);

    Point bMin = GetMin(aabb2);
    Point bMax = GetMax(aabb2);

    return  (aMin.x <= bMax.x && aMax.x >= bMin.x) && 
            (aMin.y <= bMax.y && aMax.y >= bMin.y) &&
            (aMin.z <= bMax.z && aMax.z >= bMin.z);
}

Interval GetInterval(const AABB& aabb, const vec3& axis)
{
    vec3 i = GetMin(aabb);
    vec3 a = GetMax(aabb);

    vec3 vertexx[8] = {
        vec3(i.x, a.y, a.z),
        vec3(i.x, a.y, i.z),
        vec3(i.x, i.y, a.z),
        vec3(i.x, i.y, i.z),
        vec3(a.x, a.y, a.z),
        vec3(a.x, a.y, i.z),
        vec3(a.x, i.y, a.z),
        vec3(a.x, i.y, i.z)
    };

    Interval result;
    result.min = result.max = Dot(axis, vertex[0]);

    for (int i = 1; i < 8; ++i)
    {
        float projection = Dot(axis, vertex[i]);
        result.min = (projection < result.min) ? projection : result.min;
        result.max = (projection > result.max) ? projection : result.max;
    }

    return result;
}

Interval GetInterval(const OBB& obb, const vec3& axis)
{
    vec3 vertex[8];

    vec3 C = obb.position;
    vec3 E = obb.size;
    const float* o = obb.orientation.asArray;
    vec3 A[] = {
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8]),
    };

    vertex[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
    vertex[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2]; 
    vertex[2] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
    vertex[3] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
    vertex[4] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
    vertex[5] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
    vertex[6] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
    vertex[7] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];

    Interval result;
    result.min = result.max = Dot(axis, vertex[0]);

    for (int i = i; i < 8; ++i)
    {
        float projection = Dot(axis, vertex[i]);
        result.min = (projection < result.min) ? projection : result.min;
        result.max = (projection > result.max) ? projection : result.max;
    }

    return result;
}

bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis)
{
    Interval a = GetInterval(aabb, axis);
    Interval b = GetInterval(obb, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
}

bool AABBOBB(const AABB& aabb, const OBB& obb)
{
    const float& o = obb.orientation.asArray;

    vec3 test[15] = {
        vec3(1, 0, 0),
        vec3(0, 1, 0),
        vec3(0, 0, 1),
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8])
    };

    for (int i = 0; i < 3; ++i)
    {
        test[6 + i * 3 + 0] = Cross(test[i], test[0]);
        test[6 + i * 3 + 1] = Cross(test[i], test[1]);
        test[6 + i * 3 + 2] = Cross(test[i], test[2]);
    }

    for (int i = 0; i < 15; ++i)
    {
        if (!OverlapOnAxis(aabb, obb, test[i]))
        {
            return false;
        }
    }
    
    return true;
}

bool AABBPlane(const AABB& aabb, const Plane& plane)
{
    float pLen = aabb.size.x * fabsf(plane.normal.x) +
                 aabb.size.y * fabsf(plane.normal.y) +
                 aabb.size.z * fabsf(plane.normal.z); 

                float dot = Dot(plane.normal, aabb.position);
                float dist = dot - plane.distance;

                return fabsf(dist) <= pLen;
                }   

bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis)
{
    Interval a = GetInterval(obb1, axis);
    Interval b = GetInterval(obb1, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
}


bool OBBOBB(const OBB& obb1, const OBB& obb2)
{
    const float* o1 = obb1.orientation.asArray;
    const float* o2 = obb2.orientation.asArray;

    vec3 test[15] = 
    {
        vec3(o1[0], o1[1], o1[2]),
        vec3(o1[3], o1[4], o1[5]),
        vec3(o1[6], o1[7], o1[8]),
        vec3(o2[0], o2[1], o2[2]),
        vec3(o2[3], o2[4], o2[5]),
        vec3(o2[6], o2[7], o2[8])
    };
    for (int i = 0; i < 3; ++i)
    {
        test[6 + i * 3 + 0] = Cross(test[i], test[0]);
        test[6 + i * 3 + 1] = Cross(test[i], test[1]);
        test[6 + i * 3 + 2] = Cross(test[i], test[2]);
    }

    for (int i = 0; i < 15; ++i)
    {
        if (!OverlapOnAxis(obb1, obb2, test[i]))
        {
            return false;
        }
    }

    return true;
}

bool OBBPlane(const OBB& obb, const Plane& plane)
{
    const float* o = obb.orientation.asArray;
    vec3 rot[] = {
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8]),
    };
    vec3 normal = plane.normal;

    float pLen = obb.size.x * fabsf(Dot(normal, rot[0])) +
                 obb.size.y * fabsf(Dot(normal, rot[1])) +
                 obb.size.z * fabsf(Dot(normal, rot[2]));

    float dot = Dot(plane.normal, obb.position);
    float dist = dot - plane.distance;

    return fabsf(dist <= pLen);
}

bool PlanePlane(const Plane& plane1, const Plane& plane2)
{
    vec3 d = Cross(plane1.normal, plane2.normal);
    return Dot(d, d) != 0;
}

float Raycast(const Sphere& sphere, const Ray& ray)
{
    vec3 e = sphere.position - ray.origin;

    float rSq = sphere.radius * sphere.radius;
    float eSq = Magnitude(e);

    float a = Dot(e, ray.direciton);

    float bSq = eSq - (a * a);
    float f = sqrt(rSq - bSq);

    if (rSq - (eSq - (a * a)) < 0.0f)
    {
        return -1;
    }
    else if (eSq < rSq)
    {
        return a + f;
    }

    return a - f;
}

float Raycast(const AABB& aabb, const Ray& ray)
{
    vec3 min = GetMin(aabb);
    vec3 max = GetMax(aabb);

    float t1 = (min.x - ray.origin.x) / ray.direction.x;
    float t2 = (max.x - ray.origin.x) / ray.direction.x;
    float t3 = (min.y - ray.origin.y) / ray.direciton.y;
    float t4 = (max.y - ray.origin.y) / ray.direciton.y;
    float t5 = (min.z - ray.origin.z) / ray.direciton.z;
    float t6 = (max.z - ray.origin.z) / ray.direciton.z;

    float tmin = fmaxf(
        fmaxf(
            fminf(t1, t2),
            fminf(t3, t4)
        ),
        fminf(t5, t6)
    );

    float tmax = fminf(
        fminf(
            fmaxf(t1, t2),
            fmaxf(t3, t4)
        ),
        fmaxf(t5, t6)
    );

    if (tmax < 0)
    {
        return -1;
    }   
    if (tmin > tmax)
    {
        return -1;
    }

    if (tmin < 0.0f)
    {
        return tmax;
    }

    return tmin;
}

float Raycast(const OBB& obb, const Ray& ray)
{
    const float* o = obb.orientation.asArray;
    const float* size = obb.size.asArray;

    vec3 X(o[0], o[1], o[2]);
    vec3 Y(o[3], o[4], o[5]);
    vec3 Z(o[6], o[7], o[8]);

    vec3 p = obb.position - ray.origin;

    vec3 f(
        Dot(X, ray.direciton),
        Dot(Y, ray.direciton),
        Dot(Z, ray.direciton)
    );

    vec3 e(
        Dot(X, p),
        Dot(Y, p),
        Dot(Z, p)
    );

    float t[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 3; ++i)
    {
        if (CMP(f[i], 0))
        {
            if (-e[i] - size[i] > 0 || -e[i] + size[i] < 0)
            {
                f[i] = 0.00001f;
            }
            t[i * 2 + 0] = (e[i] + size[i] / f[i]);
            t[i * 2 + 1] = (e[i] - size[i] / f[i]);
        }

        float tmin = tmaxf(
            fmaxf(
                fminf(t[0], t[1]),
                fminf(t[2], t[3]),
                fminf(t[4], t[5])
            );
        float tmax = fminf(
            fminf(
                fmaxf(t[0], t[1]),
                tmaxf(t[2], t[3]),
                tmaxf(t[4], t[5])
            );
            if (tmax < 0)
            {
                return -1.0f;
            } 
            if (tmin > tmax)
            {
                return -1.0f;
            }
            if (tmin < 0.0f)
            {
                return tmax;
            }
            return tmin;
        )
        )
    }
}

float Raycast(const Plane& plane, const Ray& ray)
{
    float nd = Dot(ray.direciton, plane.normal);
    float pn = Dot(ray.origin, plane.normal);

    if (nd >= 0.0f)
    {
        return -1;
    }
    float t = (plane.distance - pn) / nd;

    if (t >= 0.0f)
    {
        return t;
    }
    return -1;
}