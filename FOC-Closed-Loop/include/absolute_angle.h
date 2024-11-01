
class absolute_angle
{
	public:
    // absolute_angle operator=(absolute_angle a);
	long coarse = 0;	// Integer rotation
	double fine = 0;	// Angle in this rotation
	void set_abs_angle(double abs_angle);
	double operator-(absolute_angle a);
	absolute_angle operator+(absolute_angle d);
	absolute_angle operator+(double d);

	private:
	long floor_i(double in);
};