/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Simple example application showing usage of libdwt functions.
 */
#include <stdio.h> // printf

#include "libdwt.h"

int main()
{
	const int x = 512, y = 512;
	void *data1, *data2;
	int j = -1;

	// create test images
	dwt_util_alloc_image(&data1, x*sizeof(float), sizeof(float), x, y);
	dwt_util_test_image_fill_s(data1, x*sizeof(float), sizeof(float), x, y, 0);
	dwt_util_alloc_image(&data2, x*sizeof(float), sizeof(float), x, y);
	dwt_util_test_image_fill_s(data2, x*sizeof(float), sizeof(float), x, y, 0);

	// init timer
	dwt_clock_t time_start;
	dwt_clock_t time_stop;
	const int type = dwt_util_clock_autoselect();

	printf("forward transform...\n");

	// start timer
	time_start = dwt_util_get_clock(type);

	// forward transform
	dwt_cdf97_2f_s(data1, x*sizeof(float), sizeof(float), x, y, x, y, &j, 0, 0);

	// stop timer
	time_stop = dwt_util_get_clock(type);
	printf("elapsed time: %f secs\n", (double)(time_stop - time_start) / dwt_util_get_frequency(type));

	printf("inverse transform...\n");

	// start timer
	time_start = dwt_util_get_clock(type);

	// inverse transform
	dwt_cdf97_2i_s(data1, x*sizeof(float), sizeof(float), x, y, x, y, j, 0, 0);

	// stop timer
	time_stop = dwt_util_get_clock(type);
	printf("elapsed time: %f secs\n", (double)(time_stop - time_start) / dwt_util_get_frequency(type));

	// compare
	if( dwt_util_compare_s(data1, data2, x*sizeof(float), sizeof(float), x, y) )
		printf("images differs\n");
	else
		printf("success\n");

	return 0;
}
