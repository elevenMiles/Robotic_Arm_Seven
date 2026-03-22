#include "vector3.h"
#include <math.h>
#include <Arduino.h>

Vector3::Vector3(float set_x, float set_y, float set_z) : x {set_x}, y {set_y}, z {set_z}
{
}

float Vector3::magnitude() const
{
    return sqrtf(x * x + y * y + z * z);
}

float Vector3::sqrMagnitude() const
{
    return x * x + y * y + z * z;
}

Vector3 Vector3::getNormalized() const
{
    float mag {magnitude()};
    return Vector3{x / mag, y / mag, z / mag};
}

Vector3 Vector3::moveTowards(const Vector3& initial, const Vector3& target, float maxDelta)
{
    // 1. Calculate the vector pointing from current to target
    Vector3 diff {target - initial};
    
    // 2. Use squared distance to avoid a slow square root call early
    float sqDist {diff.sqrMagnitude()};

    // 3. If we are closer than the distance we can travel, just return target
    // We compare against maxDelta squared to keep the check "cheap"
    if (sqDist == 0.0f || (maxDelta >= 0.0f && sqDist <= maxDelta * maxDelta))
        return target;

    // 4. Calculate actual magnitude only once we know we need it
    float mag {sqrtf(sqDist)};

    return initial + diff * (maxDelta / mag);
}

float Vector3::sqrDistance(const Vector3& vectorA, const Vector3& vectorB)
{
    float dx {vectorA.x - vectorB.x};
    float dy {vectorA.y - vectorB.y};
    float dz {vectorA.z - vectorB.z};
    return dx * dx + dy * dy + dz * dz;
}

float Vector3::distance(const Vector3& vectorA, const Vector3& vectorB)
{
    float dx {vectorA.x - vectorB.x};
    float dy {vectorA.y - vectorB.y};
    float dz {vectorA.z - vectorB.z};
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

Vector3 Vector3::up()
{
    return {0.0f, 0.0f, 1.0f};
}

Vector3 Vector3::down()
{
    return {0.0f, 0.0f, -1.0f};
}

Vector3 Vector3::left()
{
    return {0.0f, -1.0f, 0.0f};
}

Vector3 Vector3::right()
{
    return {0.0f, 1.0f, 0.0f};
}

Vector3 Vector3::forward()
{
    return {1.0f, 0.0f, 0.0f};
}

Vector3 Vector3::backward()
{
    return {-1.0f, 0.0f, 0.0f};
}

Vector3 Vector3::zero()
{
    return {0.0f, 0.0f, 0.0f};
}

Vector3 operator+(const Vector3& vectorA, const Vector3& vectorB)
{
    return Vector3{vectorA.x + vectorB.x, vectorA.y + vectorB.y, vectorA.z + vectorB.z};
}

Vector3 operator-(const Vector3& vectorA, const Vector3& vectorB)
{
    return Vector3{vectorA.x - vectorB.x, vectorA.y - vectorB.y, vectorA.z - vectorB.z};
}

Vector3 operator*(const Vector3& vector, float scaler)
{
    return Vector3{vector.x * scaler, vector.y * scaler, vector.z * scaler};
}


void Vector3::print() const
{
    Serial.print('('); Serial.print(x); Serial.print(F(", ")); Serial.print(y); Serial.print(F(", ")); Serial.print(z); Serial.print(')');
}

void Vector3::println() const
{
    Serial.print('('); Serial.print(x); Serial.print(F(", ")); Serial.print(y); Serial.print(F(", ")); Serial.print(z); Serial.println(')');
}