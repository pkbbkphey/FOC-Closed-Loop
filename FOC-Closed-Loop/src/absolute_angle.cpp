#include "absolute_angle.h"

// absolute_angle absolute_angle::operator=(absolute_angle a)
// {
//     absolute_angle temp;
//     temp.coarse = a.coarse;
//     temp.fine = a.fine;
//     return temp;
// }

void absolute_angle::set_abs_angle(double abs_angle)
{
    coarse = floor_i(abs_angle) % 360;
    fine = abs_angle - 360 * coarse;
}

double absolute_angle::operator-(absolute_angle a)
{
    return 360.0 * (coarse - a.coarse) + (fine - a.fine);
}

absolute_angle absolute_angle::operator+(absolute_angle d)
{
    absolute_angle temp;
    temp.coarse = coarse + d.coarse;
    temp.fine = fine + d.fine;
    if(fine + d.fine >= 360.0)
    {
        ++ temp.coarse;
        temp.fine -= 360;
    }
    return temp;
}

absolute_angle absolute_angle::operator+(double d)
{
    absolute_angle temp;
    temp.set_abs_angle(d);
    temp = *this + temp;
    return temp;
}

long absolute_angle::floor_i(double in)
{
    long in_l = long(in);
    return ((in_l >= 0) ? (in_l) : (in_l - 1));
}
