/* Custom 3-D vector implementation. Supports vector addition and subtraction, 
 * scalar multiplication, normalization and magnitude of a 3-D vector.
 * 
 * Use with caution. For more complex needs, consider an Arduino-compatible Vector3 library.
 *
 */

#ifndef VECTOR3_H
#define VECTOR3_H

class Vector3
{
public:
    float x {};
    float y {};
    float z {};

    Vector3(float x = 0.0f, float y = 0.0f, float z = 0.0f);

    void print() const;
    void println() const;

    float magnitude() const;
    float sqrMagnitude() const;
    Vector3 getNormalized() const;

    // Moves initial vector in maxDelta amount to the target vector and returns result.
    static Vector3 moveTowards(const Vector3& initial, const Vector3& target, float maxDelta);

    /* Returns square of the distance between VectorA and VectorB. Because sqrt() function is 
     * computationaly expensive. You can use sqrDistance() in places where distance needs to
     * be calculated rapidly. 
     *
     * Example: 
     * if (sqrDistance(targetPosition, projectilePosition) < targetDiameter * targetDiameter)
     *      print("You hit the target.");
     */
    static float sqrDistance(const Vector3& vectorA, const Vector3& vectorB);

    // Returns the distance between vectorA and vectorB
    static float distance(const Vector3& vectorA, const Vector3& vectorB);

    /* Returns a unit vector in positive z direction. */ 
    static Vector3 up();   

    /* Returns a unit vector in negative z direction. */    
    static Vector3 down();

    /* Returns a unit vector in negative y direction. */       
    static Vector3 left();

    /* Returns a unit vector in positive y direction. */     
    static Vector3 right();

    /* Returns a unit vector in positive x direction. */    
    static Vector3 forward();

    /* Returns a unit vector in negative x direction. */  
    static Vector3 backward();

    /* Returns a vector with all 0 components. */   
    static Vector3 zero();

    friend Vector3 operator+(const Vector3& vectorA, const Vector3& vectorB);
    friend Vector3 operator-(const Vector3& vectorA, const Vector3& vectorB);
    friend Vector3 operator*(const Vector3& vector, float scaler);
};

#endif