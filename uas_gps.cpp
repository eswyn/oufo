/**
    OUFO Unmanned Aerial System
    uas_gps.cpp
    Purpose: Improves the accuracy of the raw data
             from the GPS sensor input by applying
             a Kalman filter alongside
             raw propeller data, then logs the
             flight path.

    @author Lee Cundall
    @version 1.0 20/11/18
*/

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <random>
#include <numeric>
#include <fstream>


/**
 * Simple latitude and longitude container, supports addition.
*/
class gps_datum
{
public:
	gps_datum() : latitude(0), longitude(0) { };
	gps_datum(double val_latitude, double val_longitude) : latitude(val_latitude), longitude(val_longitude) { }
	gps_datum(const gps_datum& rhs) : latitude(rhs.get_latitude() ), longitude(rhs.get_longitude() ) { }
	~gps_datum() { };
	friend std::ostream& operator<<(std::ostream& os, const gps_datum& rhs) { os << "[lat: " << rhs.get_latitude() << ", lon: " << rhs.get_longitude() << "]"; return os; }
	gps_datum& operator+(const gps_datum& rhs)
	{

		double lat_temp;
		lat_temp = latitude+rhs.get_latitude();
		if (lat_temp > 90)
		{
			double diff = 90 - latitude;
			set_latitude( 90 + diff - rhs.get_latitude());
		}
		else if (lat_temp < (-90))
		{
			double diff = 90 + latitude;
			set_latitude( (-90) - diff - rhs.get_latitude());
		}
		else
		{
			set_latitude(latitude+rhs.get_latitude());
		}

		double lon_temp;
		lon_temp = longitude+rhs.get_longitude();
		if (lon_temp > 180)
		{
			set_longitude(longitude + rhs.get_longitude() - 360);
		}
		else if (lon_temp < (-180))
		{
			set_longitude(longitude + rhs.get_longitude() + 360);
		}
		else
		{
			set_longitude(longitude+rhs.get_longitude());
		}

	}
	gps_datum& operator+=(const gps_datum& rhs)
	{ 

	}


	inline void set_latitude(double s_lat) { latitude = s_lat; }
	inline void set_longitude(double s_long) { longitude = s_long; };
	inline double get_latitude() const { return latitude; }
	inline double get_longitude() const { return longitude; }

protected:
	double latitude;
	double longitude;
};


/** 
 *	Retrives a randomized, normally distributed GPS sensor reading surrounding
 *	a simulated static position for testing purposes.
 *
 *	@param base_point Simulated GPS sensor location.
 *	@param sigma Value of standard deviation within which measurement will lie.
 *	@return Simulated GPS sensor reading.
 */
const gps_datum simulate_gps_sensor (gps_datum base_point, double sigma)
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::normal_distribution<double> nd_lat{base_point.get_latitude(), sigma};
	std::normal_distribution<double> nd_long{base_point.get_longitude(), sigma};

	const gps_datum datum(nd_lat(mt), nd_long(mt));

	return datum;
}

/** 
 *	Retrives a randomized, normally distributed speed reading for UAS
 *	which replicates propeller sensor variation.
 *
 *	@param actual_speed Actual speed of UAS in m s^-1.
 *	@param sigma Value of standard deviation within which measurement will lie.
 *	@return Simulated speed reading from pseudo-sensor.
 */
const double simulate_speed_reading (double actual_speed, double sigma)
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::normal_distribution<double> nd_speed{actual_speed, sigma};

	const double determined_speed = nd_speed(mt);

	return determined_speed;
}

/** 
 *	Calculate the distance in metres between 2 GPS coordinates
 *  
 *	@param pos1 GPS coordinates of first point
 *	@param pos2 GPS coordinates of second point
 *	@return Distance in metres between both points.
 */
double gps_to_metres (gps_datum pos1, gps_datum pos2)
{
		const double pi = 4.0 * std::atan(1.0);
		const double earth_radius_km = 6378.137;

        double lat1 = pos1.get_latitude() * (pi/180);
        double long1 = pos1.get_longitude() * (pi/180);
        double lat2 = pos2.get_latitude() * (pi/180);
        double long2 = pos2.get_longitude() * (pi/180);

        double delta_latitude = lat1 - lat2;
        double delta_longitude = long1 - long2;

        double a = std::pow( std::sin( delta_latitude / 2.0 ), 2.0 ) + std::cos(lat1) * std::cos(lat2) * std::pow( std::sin( delta_longitude / 2 ), 2);
        double c = 2 * std::atan2( std::sqrt(a), std::sqrt( 1.0 - a ) );

        double distance_m = (earth_radius_km * c) * 1000;

        return distance_m;
}

/** 
 *	Calculate the standard deviation of a static GPS sensor.
 *  
 *	@param total_duration Time in seconds to run measurement for calculation.
 *	@param n_measurements Number of measurements to take.
 *	@return The standard deviation of the GPS signal.
 */
double get_gps_stddeviation(float total_duration, short unsigned int n_measurements)
{

    using namespace std::chrono;
    long int tperiod = total_duration / n_measurements;
    steady_clock::duration time_step = duration<steady_clock::rep, std::ratio<1, 1>> {tperiod};
    steady_clock::time_point next = steady_clock::now() + time_step;

    std::vector<gps_datum> gps_readings;
    double latitude_sum = 0;
    double longitude_sum = 0;

    gps_datum test_base_point(64.2357341352, -121.235325235235);

    for (short unsigned int i = 0; i < n_measurements; i++)
    {

    	/* replace simulate_gps_sensor() with volatile GPS sensor input */
    	gps_datum current_gps_coords ( simulate_gps_sensor(test_base_point, 0.0001) );

    	gps_readings.push_back(current_gps_coords);
        std::this_thread::sleep_until(next);
        next += time_step;

		std::cout << std::setprecision(12) << gps_readings[i] << " | diff: ~" << static_cast<int> (gps_to_metres(gps_readings[i], gps_readings[((i > 0) ? (i-1) : (i))] ) ) << "m" << std::endl;

        latitude_sum += current_gps_coords.get_latitude();
        longitude_sum += current_gps_coords.get_longitude();

    }

    double latitude_mean = latitude_sum / n_measurements;
    double longitude_mean = longitude_sum / n_measurements;

    double lat_devsquare_sum = 0;
    double long_devsquare_sum = 0;

    for (std::vector<gps_datum>::iterator it = gps_readings.begin(); it != gps_readings.end(); ++it)
    {
    	double lat_deviation = it->get_latitude() - latitude_mean;
    	double long_deviation = it->get_longitude() - longitude_mean;

    	double lat_devsquare = lat_deviation * lat_deviation;
    	double long_devsquare = long_deviation * long_deviation;

    	lat_devsquare_sum += lat_devsquare;
    	long_devsquare_sum += long_devsquare;

    }

    double lat_variance = lat_devsquare_sum / (n_measurements - 1);
    double long_variance = long_devsquare_sum / (n_measurements - 1);

    double stddev = std::sqrt((lat_variance + long_variance) / 2);

    return stddev;

}

/** 
 *	Record sensor data, apply filters and record flight path
 *  
 *	@param n_seconds Number of seconds to wait between sensor readings
 */
void take_flight(unsigned int n_seconds)
{

	try
	{
		const char filename[] = "flightpath.txt";
		std::ofstream outfile;
		outfile.open(filename, std::ios::out | std::ios::trunc );

		using namespace std::chrono;
    	steady_clock::duration time_step = duration<steady_clock::rep, std::ratio<1, 1>> {n_seconds};
   		steady_clock::time_point next = steady_clock::now() + time_step;

    	std::vector<gps_datum> sensor_flightpath;
    	std::vector<gps_datum> actual_flightpath;

    	/* replace base_point values with volatile GPS sensor input */
    	gps_datum base_point(64.2357341352, -121.235325235235);
    	/* replace translate point with metres speed translate function */
    	gps_datum translate_point(10.0004135613, -10.0004131531);

    	gps_datum previous_point = base_point;

    	const unsigned n_measurements = 50;

    	std::cout << "Recording flight data to " << filename << "..." << std::endl;

    	for (short unsigned int i = 0; i < n_measurements; i++)
    	{

    		/* replace simulate_gps_sensor() with volatile GPS sensor input once available */
    		gps_datum current_gps_coords ( simulate_gps_sensor(previous_point + translate_point, 0.0001) );

    		std::cout << std::setprecision(12) << current_gps_coords << " | diff: ~" << static_cast<int> (gps_to_metres(current_gps_coords, previous_point) ) << "m" << std::endl;
    		outfile << std::setprecision(12) << current_gps_coords << " | diff: ~" << static_cast<int> (gps_to_metres(current_gps_coords, previous_point) ) << "m" << std::endl;

    		std::this_thread::sleep_until(next);
    		next += time_step;
		}

	}
	catch (std::exception const& e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		return;
	}


}


int main (int argc, char** argv)
{
  
  	double gpsdev = get_gps_stddeviation(1, 50);
  	std::cout << "GPS deviation: " << std::fixed << gpsdev << std::endl;


  	double actual_speed = 23;
  	double sensor_speed = simulate_speed_reading(actual_speed, 1);

  	std::cout << "Actual speed: " << actual_speed << std::endl;
  	std::cout << "Speed sensor: " << sensor_speed << std::endl;

  	unsigned int sensor_timestep = 2;
  	take_flight(sensor_timestep);


	return 0;
}
