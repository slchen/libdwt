/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief More complex application measuring performance.
 */

#include "libdwt.h"

// PATH_MAX
#include <limits.h>
#include <linux/limits.h>

int main(int argc, char *argv[])
{
	dwt_util_init();

	const int type = dwt_util_clock_autoselect();

	// measure up to 2000^2 pixels
	const int max_size = 2000;

	// how many threads are available
	const int max_threads = dwt_util_get_max_threads();

	const char *arr_name[] =
	{
		[DWT_ARR_SIMPLE] = "simple",
		[DWT_ARR_SPARSE] = "sparse",
		[DWT_ARR_PACKED] = "packed"
	};

	// iterate over all these parameters
	for(int decomposition = -1; decomposition <= 1; decomposition+=2)
	for(int arr = DWT_ARR_SIMPLE; arr <= DWT_ARR_PACKED; arr++)
	for(int accel = 0; accel <= 9; accel++)
	for(int opt_stride = 0; opt_stride <= 1; opt_stride++)
	for(int threads = 1; threads <= max_threads; threads++)
	{
		char file_fwd_name[PATH_MAX];
		char file_inv_name[PATH_MAX];

		sprintf(file_fwd_name, "data.trans=fwd.threads=%i.accel=%i.optstride=%i.decomp=%i.arr=%s.type=float.txt", threads, accel, opt_stride, decomposition, arr_name[arr]);
		sprintf(file_inv_name, "data.trans=inv.threads=%i.accel=%i.optstride=%i.decomp=%i.arr=%s.type=float.txt", threads, accel, opt_stride, decomposition, arr_name[arr]);
		dwt_util_log(LOG_DBG, "file_fwd_name=\"%s\"\n", file_fwd_name);
		dwt_util_log(LOG_DBG, "file_inv_name=\"%s\"\n", file_inv_name);

		FILE *file_fwd;
		FILE *file_inv;

		if( NULL == (file_fwd = fopen(file_fwd_name, "w")) )
			dwt_util_abort();
		if( NULL == (file_inv = fopen(file_inv_name, "w")) )
			dwt_util_abort();

		dwt_util_set_accel(accel);
		dwt_util_set_num_threads(threads);

		dwt_util_measure_perf_cdf97_2_s(
			DWT_ARR_PACKED,
			1, /* min */
			max_size, /* max */
			opt_stride, /* opt_stride */
			decomposition, /* decomposition */
			1,
			0,
			1, /* the number of transforms in each test loop, avg. is calculated */
			8, /* the number of test loops, minimum is selected */
			type,
			file_fwd,
			file_inv
		);

		fclose(file_fwd);
		fclose(file_inv);
	}

	dwt_util_finish();

	return 0;
}
