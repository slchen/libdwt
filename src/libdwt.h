/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Fast wavelet transform implemented via lifting scheme.
 */
#ifndef LIBDWT_H
#define LIBDWT_H

#ifndef _POSIX_C_SOURCE
	#define _POSIX_C_SOURCE 199309L
#endif
#ifndef _GNU_SOURCE
	#define _GNU_SOURCE
#endif

#include <stdint.h> // int64_t

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup c_dwt C interface
 * @{
 **/

/**
 * @brief Lifting implementation of one level of fast wavelet transform.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 *
 * @deprecated Use @ref dwt_cdf97_f_ex_d instead.
 */
void dwt_cdf97_f_d(
	const double *src,	///< input signal of the length @e N
	double *dst,		///< output signal of the length @e N, i.e. one level of discrete wavelet transform, the L and H channels are concatenated
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @deprecated Use @ref dwt_cdf97_f_ex_s instead.
 */
void dwt_cdf97_f_s(
	const float *src,	///< input signal of the length @e N
	float *dst,		///< output signal of the length @e N, i.e. one level of discrete wavelet transform, the L and H channels are concatenated
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 *
 * @deprecated Use @ref dwt_cdf97_i_ex_d instead.
 */
void dwt_cdf97_i_d(
	const double *src,	///< input signal of the length @e N, i.e. the discrete wavelet transform
	double *dst,		///< output signal of the length @e N, i.e. the reconstructed signal
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @deprecated Use @ref dwt_cdf97_i_ex_s instead.
 */
void dwt_cdf97_i_s(
	const float *src,	///< input signal of the length @e N, i.e. the discrete wavelet transform
	float *dst,		///< output signal of the length @e N, i.e. the reconstructed signal
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_f_ex_d(
	const double *src,	///< input signal of the length @e N
	double *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	double *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the input signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_f_ex_s(
	const float *src,	///< input signal of the length @e N
	float *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	float *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the input signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform performed on column.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_f_ex_stride_d(
	const double *src,	///< input signal of the length @e N
	double *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	double *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the input signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes between two neighboring pixels, no matter if in the row or column, common for @e src, @e dst_l and @e dst_h
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform performed on column.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_f_ex_stride_s(
	const float *src,	///< input signal of the length @e N
	float *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	float *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the input signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes between two neighboring pixels, no matter if in the row or column, common for @e src, @e dst_l and @e dst_h
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_i_ex_d(
	const double *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const double *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *dst,		///< reconstructed (output) signal
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the reconstructed (output) signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_i_ex_s(
	const float *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const float *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *dst,		///< reconstructed (output) signal
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the reconstructed (output) signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_i_ex_stride_d(
	const double *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const double *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *dst,		///< reconstructed (output) signal
	double *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the reconstructed (output) signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst, @e src_l and @e src_h
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_i_ex_stride_s(
	const float *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const float *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *dst,		///< reconstructed (output) signal
	float *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the reconstructed (output) signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst, @e src_l and @e src_h
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_zero_padding_f_d(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	double *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H		///< length of the H channel
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_zero_padding_f_s(
	float *dst_l,		///< L (low pass) channel which will be padded with zeros
	float *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H		///< length of the H channel
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_zero_padding_f_stride_d(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	double *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H,		///< length of the H channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst_l and @e dst_h
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_zero_padding_f_stride_s(
	float *dst_l,		///< L (low pass) channel which will be padded with zeros
	float *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H,		///< length of the H channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst_l and @e dst_h
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_zero_padding_i_d(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst		///< total length of the L channel
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_zero_padding_i_s(
	float *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst		///< total length of the L channel
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_zero_padding_i_stride_d(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst,		///< total length of the L channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_zero_padding_i_stride_s(
	float *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst,		///< total length of the L channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels
);

/**
 * @brief Forward image fast wavelet transform using CDF 9/7 wavelet and lifting scheme, in-place version.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_2f_d(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int *j_max_ptr,		///< pointer to the number of intended decomposition levels (scales), the number of achieved decomposition levels will be stored also here
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Forward image fast wavelet transform using CDF 9/7 wavelet and lifting scheme, in-place version.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_2f_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int *j_max_ptr,		///< pointer to the number of intended decomposition levels (scales), the number of achieved decomposition levels will be stored also here
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Inverse image fast wavelet transform using CDF 9/7 wavelet and lifting scheme, in-place version.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_2i_d(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< pointer to the number of achieved decomposition levels (scales)
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Inverse image fast wavelet transform using CDF 9/7 wavelet and lifting scheme, in-place version.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_2i_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< pointer to the number of achieved decomposition levels (scales)
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @}
 */

/**
 * @defgroup c_dwt_util C utility interface
 * @{
 **/

/**
 * @brief Smallest integer not less than the base 2 logarithm of x, i.e. @f$ \lceil log_2(x) \rceil @f$.
 */
int dwt_util_ceil_log2(
	int x			///< input value
);

/**
 * @brief Power of two using greater or equal to x, i.e. @f$ 2^{\lceil log_2(x) \rceil} @f$.
 */
int dwt_util_pow2_ceil_log2(
	int x			///< input value
);

/**
 * @brief Fill test image.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 *
 * @warning experimental
 */
void dwt_util_test_image_fill_d(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int rand		///< random seed
);

/**
 * @brief Fill test image.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @warning experimental
 */
void dwt_util_test_image_fill_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int rand		///< random seed
);

/**
 * @brief Allocate image.
 *
 * Allocates memory for image of given sizes.
 *
 * @warning experimental
 */
void dwt_util_alloc_image(
	void **pptr,		///< place pointer to newly allocated data here
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y	///< height of outer image frame (in elements)
);

/**
 * @brief Free image.
 *
 * Frees memory allocated by @ref dwt_util_alloc_image.
 *
 * @warning experimental
 */
void dwt_util_free_image(
	void **pptr		///< pointer to data that will be released
);

/**
 * @brief Compare two images.
 *
 * This function compares two images and returns zero value if they equal.
 * This function works with double precision floating point numbers (i.e. double data type).
 *
 * @warning experimental
 */
int dwt_util_compare_d(
	void *ptr1,		///< pointer to data of first image
	void *ptr2,		///< pointer to data of second image
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @brief Compare two images.
 *
 * This function compares two images and returns zero value if they equal.
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @warning experimental
 */
int dwt_util_compare_s(
	void *ptr1,		///< pointer to data of first image
	void *ptr2,		///< pointer to data of second image
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @brief Returns library version string.
 */
const char *dwt_util_version();

/**
 * @brief Timer types.
 *
 * Timer sources used in @ref dwt_util_get_frequency and @ref dwt_util_get_clock functions.
 *
 * @warning experimental
 */
enum
{
	DWT_TIME_CLOCK_GETTIME,		///< use clock_gettime() function from time.h; defined by POSIX
	DWT_TIME_CLOCK,			///< use clock() function from time.h; defined since C89 and by POSIX
	DWT_TIME_TIMES,			///< use times() function from sys/times.h; defined by POSIX
	DWT_TIME_GETRUSAGE,		///< use getrusage() function from sys/resource.h; defined by POSIX
	DWT_TIME_AUTOSELECT		///< autoselect appropriate timer
};

/**
 * @brief Integer type for storing time or number of clock ticks.
 *
 * This type is used by @ref dwt_util_get_frequency and @ref dwt_util_get_clock functions.
 *
 * @warning experimental
 */
typedef int64_t dwt_clock_t;

/**
 * @brief Autoselect appropriate timer.
 *
 * Try to select appropriate timer according to system dispositions.
 *
 * @warning experimental
 */
int dwt_util_clock_autoselect();

/**
 * @brief Number of ticks per second.
 *
 * This function returns a number of clock ticks per second according to indicated clock source type.
 *
 * @warning experimental
 */
dwt_clock_t dwt_util_get_frequency(
	int type);

/**
 * @brief Number of ticks from certain event.
 *
 * This function returns a number of clock ticks from some event (e.g. system boot). Used clock source is indicated by @p type argument.
 *
 * Example usage:
 * @code
 * int type = dwt_util_clock_autoselect();
 *
 * dwt_clock_t start = dwt_util_get_clock(type);
 * // some computation...
 * dwt_clock_t stop = dwt_util_get_clock(type);
 *
 * double elapsed_time_in_seconds = (stop - start)/(double)dwt_util_get_frequency(type);
 * @endcode
 *
 * @warning experimental
 */
dwt_clock_t dwt_util_get_clock(
	int type);

/**
 * @brief Wrapper to @p omp_get_max_threads function.
 *
 * Returns the maximum number of threads what can be used in parallel region.
 *
 * @warning experimental
 */
int dwt_util_get_max_threads();

/**
 * @brief Wrapper to @p omp_set_num_threads function.
 *
 * Sets the number of threads that will be used in parallel region.
 *
 * @warning experimental
 */
void dwt_util_set_num_threads(
	int num_threads);

/**
 * @brief Enable block-acceleration using workers in UTIA EdkDSP platform.
 *
 * @warning highly experimental
 */
void dwt_util_set_accel(
	int accel_type);

/**
 * @brief Initialize worker in UTIA EdkDSP platform.
 *
 * @warning highly experimental
 */
void dwt_util_init();

/**
 * @brief Release all resources allocated in @ref dwt_util_init function.
 *
 * @warning highly experimental
 */
void dwt_util_finish();

/**
 * @brief Save grayscale image into PGM file.
 *
 * See <a href="http://netpbm.sourceforge.net/">the home page for Netpbm</a>.
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @warning highly experimental
 */
int dwt_util_save_to_pgm_s(
	const char *filename,	///< target file name, e.g. "output.pgm"
	float max_value, 	///< maximum value of pixel, e.g. 1.0 if image values lie inside an interval [0.0; 1.0]
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
