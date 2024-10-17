#include "bessel_filter.h"

///////////////////////////////////////////////////////////
bessel_filter::bessel_filter(QWidget *parent, int length_of_vector_to_be_smoothed_in):
QWidget(parent)
{
	length_of_vector_to_be_smoothed = length_of_vector_to_be_smoothed_in;
	signal_out_smoothed = std::vector<double>(length_of_vector_to_be_smoothed,0);
	signal_in_raw = std::vector<double>(length_of_vector_to_be_smoothed,0);
   
	////////////////////////////////////////////////////Zero these variables to eliminate incorrect starting blips in the data.
	for(int i = 0; i < bessel_order + 1; i++)
	{
		FT_raw_vec.push_back(std::vector<double>(length_of_vector_to_be_smoothed, 0));
		FT_filtered_vec.push_back(std::vector<double>(length_of_vector_to_be_smoothed, 0));
	}
	////////////////////////////////////////////////////
}
///////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
std::vector<double> bessel_filter::update(std::vector<double> vec)
{
    long double a[bessel_order+1] = {0.0000,   -1.9485,    0.9494};
    long double b[bessel_order+1] = {0.2193,    0.4385,    0.2193};

	signal_in_raw = vec;

    for(int axis_counter = 0; axis_counter <= signal_in_raw.size()-1; axis_counter++)
    {
            //shift data
            for(int memory_counter = bessel_order; memory_counter >= 1; memory_counter--)
            {
				FT_raw_vec[memory_counter][axis_counter] = FT_raw_vec[memory_counter-1][axis_counter];
                FT_filtered_vec[memory_counter][axis_counter] = FT_filtered_vec[memory_counter-1][axis_counter];
            }

			//Update newest data point
            FT_raw_vec[0][axis_counter] = signal_in_raw[axis_counter];
            FT_filtered_vec[0][axis_counter] = 0;
            for(int k = 0; k <= bessel_order; k++)
            {
				FT_filtered_vec[0][axis_counter] = FT_filtered_vec[0][axis_counter] + (1e-3)*b[k]*FT_raw_vec[k][axis_counter] - a[k]*FT_filtered_vec[k][axis_counter];
            }

            signal_out_smoothed[axis_counter] = FT_filtered_vec[0][axis_counter];
    }

	return signal_out_smoothed;
}
///////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
void bessel_filter::timer_update(QTimerEvent *event)
{
   
}
///////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
void bessel_filter::close(void)
{
   
}
///////////////////////////////////////////////////////////