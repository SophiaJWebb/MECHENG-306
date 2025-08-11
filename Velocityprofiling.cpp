// input: Xi and Yi coordinates of a point
// Xf and Yf coordinates of the final point
// Maximum velocity
// output: Velocity profile for the point


#include <cmath>
#include <iostream>

int main() {
	double Xi, Yi, Xf, Yf;
	double velocity_max; // Maximum velocity
	double acceleration; // Acceleration rate

	// Example initialization (replace with actual values or user input)
	Xi = 0.0;
	Yi = 0.0;
	Xf = 10.0;
	Yf = 0.0;
	velocity_max = 5.0;
	acceleration = 2.0;

	double acceleration_time = velocity_max / acceleration;
	double distance_covered = 0.5 * acceleration * acceleration_time * acceleration_time; // Distance covered during acceleration

	double distance_total = sqrt((Xf - Xi) * (Xf - Xi) + (Yf - Yi) * (Yf - Yi)); // Total distance to the final point


    // Check if the distance to be covered is less than or equal to the distance covered during acceleration
	if (2 * distance_covered <= distance_total)
	{
        // trapezoidal velocity profile
		double cruise_distance = distance_total - 2 * distance_covered; // Distance to be covered at maximum velocity
		double cruise_time = cruise_distance / velocity_max; // Time to cover the cruise distance at maximum velocity
		double total_time = 2 * acceleration_time + cruise_time; // Total time for the velocity profile
	} else
    {
        // triangular velocity profile
        double velocity_max_effective = sqrt(acceleration * distance_total); // Effective maximum velocity based on distance
        double total_time = 2 * velocity_max_effective / acceleration; // Total time for the velocity profile
    }
    double velocity_profile = 0.0; // Placeholder for velocity profile output
    // Output the velocity profile (for demonstration purposes)
}
