#ifndef __DEAD_RECKONING__
#define __DEAD_RECKONING__

class Posture
{
    private :

        float x_;
        float y_;
        float theta_;

    public :

        Posture(float x, float y, float theta);

        Posture operator +(Posture);
        Posture operator -(Posture);
        Posture operator *(float);
        Posture operator /(float);

        void operator +=(Posture);
        void operator -=(Posture);
        void operator *=(float);
        void operator /=(float);
};

Posture operator *(float, Posture);
Posture operator /(float, Posture);

class velocity
{
    public:

        float v;
        float omega;

        velocity(float v, float omega);

        Posture operator +(velocity);
        Posture operator -(velocity);
        Posture operator *(float);
        Posture operator /(float);

        void operator +=(velocity);
        void operator -=(velocity);
        void operator *=(float);
        void operator /=(float);
};

Posture operator *(float, velocity);
Posture operator /(float, velocity);

class Jacobian
{
    private:
        float cos_theta;
        float sin_theta;
    public:
        Jacobian(Posture);
        Posture operator *(velocity);
};

Posture explicit_update(Posture p, velocity q, float Dt)
{
    return p + Jacobian(p) * q * Dt;
}

Posture implicit_update(Posture p, velocity q, float Dt)
{

}

#endif