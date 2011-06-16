/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Simple example application showing usage of libdwt functions.
 */
#include <stdio.h> // printf

#include "libdwt.h"

int main()
{
	dwt_util_init();

	printf("version=\"%s\"\n", dwt_util_version());

	const int x = 512, y = 512;
	const int stride_x = x*sizeof(float), stride_y = sizeof(float);
	void *data1, *data2;
	int j = -1;

	printf("generating test images...\n");

	// create test images
	dwt_util_alloc_image(&data1, stride_x, stride_y, x, y);
	dwt_util_test_image_fill_s(data1, stride_x, stride_y, x, y, 0);
	dwt_util_alloc_image(&data2, stride_x, stride_y, x, y);
	dwt_util_test_image_fill_s(data2, stride_x, stride_y, x, y, 0);

	// init timer
	dwt_clock_t time_start;
	dwt_clock_t time_stop;
	const int type = dwt_util_clock_autoselect();

	printf("forward transform...\n");

	// start timer
	time_start = dwt_util_get_clock(type);

	// forward transform
	dwt_cdf97_2f_s(data1, stride_x, stride_y, x, y, x, y, &j, 0, 0);

	// stop timer
	time_stop = dwt_util_get_clock(type);
	printf("elapsed time: %f secs\n", (double)(time_stop - time_start) / dwt_util_get_frequency(type));

	printf("inverse transform...\n");

	// start timer
	time_start = dwt_util_get_clock(type);

	// inverse transform
	dwt_cdf97_2i_s(data1, stride_x, stride_y, x, y, x, y, j, 0, 0);

	// stop timer
	time_stop = dwt_util_get_clock(type);
	printf("elapsed time: %f secs\n", (double)(time_stop - time_start) / dwt_util_get_frequency(type));

	// compare
	if( dwt_util_compare_s(data1, data2, stride_x, stride_y, x, y) )
		printf("images differs\n");
	else
		printf("success\n");

	printf("saving...\n");
	dwt_util_save_to_pgm_s("data1.pgm", 1.0, data1, stride_x, stride_y, x, y);
	dwt_util_save_to_pgm_s("data2.pgm", 1.0, data2, stride_x, stride_y, x, y);

	dwt_util_finish();

	return 0;
}
