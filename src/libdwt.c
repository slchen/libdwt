/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Fast wavelet transform implemented via lifting scheme.
 */
#include "libdwt.h"

#ifdef microblaze
	#include <wal.h>
	#include <wal_bce_jk.h>
	#include <bce_fp01_1x1_plbw.h>

	WAL_REGISTER_WORKER(worker, BCE_JK_FP32M24, BCE_FP01_1X1_PLBW_ConfigTable, 0, 1, 0);

	#include "firmware/fw_fp01_lift4sa.h"
	#include "firmware/fw_fp01_lift4sb.h"
#endif

/**
 * Size of each EdkDSP memory bank is 256 floats
 */
#ifdef microblaze
	#define BANK_SIZE 256
#else
	#define BANK_SIZE 256
#endif

#ifdef microblaze

#ifdef NDEBUG
	#define WAL_CHECK(expr) (expr)
#else
	#define WAL_CHECK(expr) ( (expr) ? abort() : (void)0 )
#endif

#define WAL_BANK_POS(bank, off) ( (bank)*BANK_SIZE + (off) )

#endif

#if !defined(P4A)
	#define USE_TIME_CLOCK
	#define USE_TIME_CLOCK_GETTIME
	#define USE_TIME_TIMES
	#define USE_TIME_GETRUSAGE
#endif

#if defined(_GNU_SOURCE) || defined(_ISOC99_SOURCE) || defined(_POSIX_C_SOURCE)
	#define HAVE_TIME_CLOCK
#endif

#if _POSIX_C_SOURCE >= 199309L || _XOPEN_SOURCE >= 500
	#define HAVE_TIME_CLOCK_GETTIME
#endif

#if defined(_GNU_SOURCE) || defined(_SVID_SOURCE) || defined(_BSD_SOURCE) || defined(_POSIX_C_SOURCE)
	#define HAVE_TIME_TIMES
#endif

#if defined(_GNU_SOURCE) || defined(_SVID_SOURCE) || defined(_BSD_SOURCE) || defined(_POSIX_C_SOURCE)
	#define HAVE_TIME_GETRUSAGE
#endif

#if defined(USE_TIME_CLOCK) && defined(HAVE_TIME_CLOCK)
	#define ENABLE_TIME_CLOCK
#endif

#if defined(USE_TIME_CLOCK_GETTIME) && defined(HAVE_TIME_CLOCK_GETTIME)
	#define ENABLE_TIME_CLOCK_GETTIME
#endif

#if defined(USE_TIME_TIMES) && defined(HAVE_TIME_TIMES)
	#define ENABLE_TIME_TIMES
#endif

#if defined(USE_TIME_GETRUSAGE) && defined(HAVE_TIME_GETRUSAGE)
	#define ENABLE_TIME_GETRUSAGE
#endif

#ifdef ENABLE_TIME_CLOCK_GETTIME
	#include <time.h> // FIXME: -lrt
#endif

#ifdef ENABLE_TIME_CLOCK
	#include <time.h>
#endif

#ifdef ENABLE_TIME_TIMES
	#include <sys/times.h>
	#include <unistd.h>
#endif

#ifdef ENABLE_TIME_GETRUSAGE
	#include <time.h>
	#include <unistd.h>
	#include <sys/resource.h>
	#include <sys/time.h>
#endif

#include <assert.h> // assert
#include <stddef.h> // NULL, size_t
#include <stdlib.h> // abort, malloc, free
#include <limits.h> // CHAR_BIT
#include <math.h> // fabs, fabsf, isnan, isinf, FIXME: -lm
#include <stdio.h> // FILE, fopen, fprintf, fclose
#include <string.h> // memcpy

#ifdef _OPENMP
	#include <omp.h>
#endif

#define UNUSED(expr) do { (void)(expr); } while (0)

#ifndef PACKAGE_NAME
	#error PACKAGE_NAME is not defined
#endif

#ifndef PACKAGE_VERSION
	#error PACKAGE_VERSION is not defined
#endif

#define __Q(x) #x
#define _Q(x) __Q(x)
#define LIBDWT_VERSION_STRING _Q(PACKAGE_NAME) " " _Q(PACKAGE_VERSION)

const char *dwt_util_version()
{
	return LIBDWT_VERSION_STRING;
}

/**
 * @brief Global variable indicating used acceleration type.
 * @note Array of size of 2 is workaround due to GCC 3.4.1 bug.
 */
int dwt_util_global_accel_type[2] = {0};

/**
 * @{
 * @brief CDF 9/7 lifting scheme constants
 * These constants are found in S. Mallat. A Wavelet Tour of Signal Processing: The Sparse Way (Third Edition). 3rd edition, 2009 on page 370.
 */
static const double dwt_cdf97_p1_d =    1.58613434342059;
static const double dwt_cdf97_u1_d =   -0.0529801185729;
static const double dwt_cdf97_p2_d =   -0.8829110755309;
static const double dwt_cdf97_u2_d =    0.4435068520439;
static const double dwt_cdf97_s1_d =    1.1496043988602;
static const double dwt_cdf97_s2_d =  1/1.1496043988602; // TODO: unnecessary

static const float dwt_cdf97_p1_s =    1.58613434342059;
static const float dwt_cdf97_u1_s =   -0.0529801185729;
static const float dwt_cdf97_p2_s =   -0.8829110755309;
static const float dwt_cdf97_u2_s =    0.4435068520439;
static const float dwt_cdf97_s1_s =    1.1496043988602;
static const float dwt_cdf97_s2_s =  1/1.1496043988602; // TODO: unnecessary
/**@}*/

/**
 * @brief Power of two using greater or equal to x, i.e. 2^(ceil(log_2(x)).
 */
static
int pow2_ceil_log2(
	int x)
{
	assert(x > 0);

	x--;

	unsigned shift = 1;

	while(shift < sizeof(int) * CHAR_BIT)
	{
		x |= x >> shift;
		shift <<= 1;
	}

	x++;

	return x;
}

/**
 * @brief Number of 1-bits in x, in parallel.
 */
static
int bits(
	unsigned x)
{
	x -= x >> 1 & (unsigned)~(unsigned)0/3;
	x = (x & (unsigned)~(unsigned)0/15*3) + (x >> 2 & (unsigned)~(unsigned)0/15*3);
	x = (x + (x >> 4)) & (unsigned)~(unsigned)0/255*15;
	return (x * ((unsigned)~(unsigned)0/255)) >> (sizeof(unsigned) - 1) * CHAR_BIT;
}

/**
 * @brief Smallest integer not less than the base 2 logarithm of x, i.e. ceil(log_2(x)).
 * @returns (int)ceil(log2(x))
 */
static
int ceil_log2(
	int x)
{
	return bits(pow2_ceil_log2(x) - 1);
}

int dwt_util_ceil_log2(
	int x)
{
	return ceil_log2(x);
}

int dwt_util_pow2_ceil_log2(
	int x)
{
	return pow2_ceil_log2(x);
}

/**
 * @returns (int)ceil(x/(double)y)
 */
static
int ceil_div(
	int x,
	int y)
{
	return (x + y - 1) / y;
}

/**
 * @returns (int)ceil(i/(double)(1<<j))
 */
static
int ceil_div_pow2(
	int i,
	int j)
{
	return (i + (1 << j) - 1) >> j;
}

/** @returns (int)floor(x/(double)2) */
static
int floor_div2(
	int x)
{
	return x >> 1;
}

/** @returns (int)ceil(x/(double)2) */
static
int ceil_div2(
	int x)
{
	return (x+1) >> 1;
}

/**
 * @brief Minimum of two integers.
 */
static
int min(
	int a,
	int b)
{
	return a>b ? b : a;
}

/**
 * @brief Maximum of two integers.
 */
static
int max(
	int a,
	int b)
{
	return a>b ? a : b;
}

/** returns 1 if x is odd, 0 otherwise */
static
int is_odd(
	int x)
{
	return x&1;
}

/** returns 1 if x is even, 0 otherwise */
static
int is_even(
	int x)
{
	return 1&~x;
}

/** returns closest even integer not larger than x */
static
int to_even(
	int x)
{
	return x&~1;
}

/** returns closest odd integer not larger than x */
static
int to_odd(
	int x)
{
	return x - (1&~x);
}

/**
 * @brief Helper function returning address of given element.
 *
 * Evaluate address of (i) image element, returns (const double *).
 */
static
const double *addr1_const_d(
	const void *ptr,
	int i,
	int stride)
{
	return (const double *)((const char *)ptr+i*stride);
}

/**
 * @brief Helper function returning address of given element.
 *
 * Evaluate address of (i) image element, returns (const double *).
 */
static
const float *addr1_const_s(
	const void *ptr,
	int i,
	int stride)
{
	return (const float *)((const char *)ptr+i*stride);
}

/**
 * @brief Helper function returning address of given element.
 *
 * Evaluate address of (i) image element, returns (double *).
 */
static
double *addr1_d(
	void *ptr,
	int i,
	int stride)
{
	return (double *)((char *)ptr+i*stride);
}

/**
 * @brief Helper function returning address of given element.
 *
 * Evaluate address of (i) image element, returns (double *).
 */
static
float *addr1_s(
	void *ptr,
	int i,
	int stride)
{
	return (float *)((char *)ptr+i*stride);
}

/**
 * @brief Helper function returning address of given pixel.
 *
 * Evaluate address of (x,y) image element, returns (double *).
 */
static
double *addr2_d(
	void *ptr,
	int y,
	int x,
	int stride_x,
	int stride_y)
{
	return (double *)((char *)ptr+y*stride_x+x*stride_y);
}

/**
 * @brief Helper function returning address of given pixel.
 *
 * Evaluate address of (x,y) image element, returns (double *).
 */
static
float *addr2_s(
	void *ptr,
	int y,
	int x,
	int stride_x,
	int stride_y)
{
	return (float *)((char *)ptr+y*stride_x+x*stride_y);
}

/**
 * @brief Copy memory area.
 *
 * This function copies @p n floats from memory area @p src to memory area
 * @p dst. Memory areas can be sparse. The strides (in bytes) are determined by
 * @p stride_dst and @p stride_src arguments.
 *
 * @returns The function returns a pointer to @p dst.
 */
static
void *dwt_util_memcpy_stride_s(
	void *dst,
	ssize_t stride_dst,
	const void *src,
	ssize_t stride_src,
	size_t n		///< Number of floats to be copied, not number of bytes.
	)
{
	// TODO: assert

	const size_t size = sizeof(float);

	if( (ssize_t)size == stride_src && (ssize_t)size == stride_dst )
	{
		memcpy(dst, src, n*size);
	}
	else
	{
		char *ptr_dst = (char *)dst;
		const char *ptr_src = (const char *)src;
		for(size_t i = 0; i < n; i++)
		{
			*(float *)ptr_dst = *(const float *)ptr_src;
	
			ptr_dst += stride_dst;
			ptr_src += stride_src;
		}
	}

	return dst;
}

/**
 * @brief Copy memory area.
 *
 * This function copies @p n doubles from memory area @p src to memory area
 * @p dst. Memory areas can be sparse. The strides (in bytes) are determined by
 * @p stride_dst and @p stride_src arguments.
 *
 * @returns The function returns a pointer to @p dst.
 */
static
void *dwt_util_memcpy_stride_d(
	void *dst,
	ssize_t stride_dst,
	const void *src,
	ssize_t stride_src,
	size_t n		///< Number of doubles to be copied, not number of bytes.
	)
{
	// TODO: assert

	const size_t size = sizeof(double);

	if( (ssize_t)size == stride_src && (ssize_t)size == stride_dst )
	{
		memcpy(dst, src, n*size);
	}
	else
	{
		char *ptr_dst = (char *)dst;
		const char *ptr_src = (const char *)src;
		for(size_t i = 0; i < n; i++)
		{
			*(double *)ptr_dst = *(const double *)ptr_src;
	
			ptr_dst += stride_dst;
			ptr_src += stride_src;
		}
	}

	return dst;
}

/**
 * @brief Pixel value of test image.
 */
static
double dwt_util_test_image_value_d(
	int x,
	int y,
	int rand)
{
	x >>= rand;
	return 2*x*y / (double)(x*x + y*y + 1);
}

/**
 * @brief Pixel value of test image.
 */
static
float dwt_util_test_image_value_s(
	int x,
	int y,
	int rand)
{
	x >>= rand;
	return 2*x*y / (float)(x*x + y*y + 1);
}

void dwt_util_test_image_fill_d(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_i_big_x,
	int size_i_big_y,
	int rand)
{
	// TODO: assert

	for(int y = 0; y < size_i_big_y; y++)
		for(int x = 0; x < size_i_big_x; x++)
			*addr2_d(ptr, y, x, stride_x, stride_y) = dwt_util_test_image_value_d(x, y, rand);
}

void dwt_util_test_image_fill_s(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_i_big_x,
	int size_i_big_y,
	int rand)
{
	// TODO: assert

	for(int y = 0; y < size_i_big_y; y++)
		for(int x = 0; x < size_i_big_x; x++)
			*addr2_s(ptr, y, x, stride_x, stride_y) = dwt_util_test_image_value_s(x, y, rand);
}

void dwt_util_alloc_image(
	void **pptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y)
{
	// TODO: assert

	UNUSED(stride_y);
	UNUSED(size_o_big_x);

	*pptr = malloc(stride_x*size_o_big_y);
	if(NULL == *pptr)
		abort();
}

void dwt_util_free_image(
	void **pptr)
{
	// TODO: assert

	free(*pptr);
	*pptr = NULL;
}

int dwt_util_compare_d(
	void *ptr1,
	void *ptr2,
	int stride_x,
	int stride_y,
	int size_i_big_x,
	int size_i_big_y)
{
	// TODO: assert

	const double eps = 1e-6;

	for(int y = 0; y < size_i_big_y; y++)
		for(int x = 0; x < size_i_big_x; x++)
		{
			const double a = *addr2_d(ptr1, y, x, stride_x, stride_y);
			const double b = *addr2_d(ptr2, y, x, stride_x, stride_y);

			if( isnan(a) || isinf(a) || isnan(b) || isinf(b) )
				return 1;

			if( fabs(a - b) > eps )
				return 1;
		}

	return 0;
}

int dwt_util_compare_s(
	void *ptr1,
	void *ptr2,
	int stride_x,
	int stride_y,
	int size_i_big_x,
	int size_i_big_y)
{
	// TODO: assert

	const float eps = 1e-3;

	for(int y = 0; y < size_i_big_y; y++)
		for(int x = 0; x < size_i_big_x; x++)
		{
			const float a = *addr2_s(ptr1, y, x, stride_x, stride_y);
			const float b = *addr2_s(ptr2, y, x, stride_x, stride_y);

			if( isnan(a) || isinf(a) || isnan(b) || isinf(b) )
				return 1;

			if( fabsf(a - b) > eps )
				return 1;
		}

	return 0;
}

void dwt_cdf97_f_d(
	const double *src,
	double *dst,
	double *tmp,
	int N)
{
	dwt_cdf97_f_ex_d(
		src,
		dst,
		dst + ceil_div2(N),
		tmp,
		N
	);
}

void dwt_cdf97_f_s(
	const float *src,
	float *dst,
	float *tmp,
	int N)
{
	dwt_cdf97_f_ex_s(
		src,
		dst,
		dst + ceil_div2(N),
		tmp,
		N
	);
}

void dwt_cdf97_i_d(
	const double *src,
	double *dst,
	double *tmp,
	int N)
{
	dwt_cdf97_i_ex_d(
		src,
		src + ceil_div2(N),
		dst,
		tmp,
		N
	);
}

void dwt_cdf97_i_s(
	const float *src,
	float *dst,
	float *tmp,
	int N)
{
	dwt_cdf97_i_ex_s(
		src,
		src + ceil_div2(N),
		dst,
		tmp,
		N
	);
}

void dwt_cdf97_f_ex_d(
	const double *src,
	double *dst_l,
	double *dst_h,
	double *tmp,
	int N)
{
	dwt_cdf97_f_ex_stride_d(
		src,
		dst_l,
		dst_h,
		tmp,
		N,
		sizeof(double)
	);
}

void dwt_cdf97_f_ex_s(
	const float *src,
	float *dst_l,
	float *dst_h,
	float *tmp,
	int N)
{
	dwt_cdf97_f_ex_stride_s(
		src,
		dst_l,
		dst_h,
		tmp,
		N,
		sizeof(float)
	);
}

void dwt_cdf97_f_ex_stride_d(
	const double *src,
	double *dst_l,
	double *dst_h,
	double *tmp,
	int N,
	int stride)
{
	assert(!( N < 0 || NULL == src || NULL == dst_l || NULL == dst_h || NULL == tmp || 0 == stride ));

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst_l[0] = src[0] * dwt_cdf97_s1_d;
		return;
	}

	// copy src into tmp
	dwt_util_memcpy_stride_d(tmp, sizeof(double), src, stride, N);

	// predict 1 + update 1
	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] -= dwt_cdf97_p1_d * (tmp[i-1] + tmp[i+1]);

	if(is_odd(N))
		tmp[N-1] += 2 * dwt_cdf97_u1_d * tmp[N-2];
	else
		tmp[N-1] -= 2 * dwt_cdf97_p1_d * tmp[N-2];

	tmp[0] += 2 * dwt_cdf97_u1_d * tmp[1];

	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] += dwt_cdf97_u1_d * (tmp[i-1] + tmp[i+1]);

	// predict 2 + update 2
	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] -= dwt_cdf97_p2_d * (tmp[i-1] + tmp[i+1]);

	if(is_odd(N))
		tmp[N-1] += 2 * dwt_cdf97_u2_d * tmp[N-2];
	else
		tmp[N-1] -= 2 * dwt_cdf97_p2_d * tmp[N-2];

	tmp[0] += 2 * dwt_cdf97_u2_d * tmp[1];

	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] += dwt_cdf97_u2_d * (tmp[i-1] + tmp[i+1]);

	// scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s1_d;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s2_d;

	// copy tmp into dst
	dwt_util_memcpy_stride_d(dst_l, stride, tmp+0, 2*sizeof(double),  ceil_div2(N));
	dwt_util_memcpy_stride_d(dst_h, stride, tmp+1, 2*sizeof(double), floor_div2(N));
}

/**
 * @brief Non-accelerated PicoBlaze operation.
 *
 * Two pairs (predict and update) of lifting steps and coefficients scaling
 * merged together.
 *
 * @param[in] scaling Perform scaling of coefficients. Possible values are:
 *   @li s = 0 : without scaling,
 *   @li s > 0 : scaling after lifting,
 *   @li s < 0 : scaling before lifting.
 */
static
void accel_lift_op4s_main_s(
	float *arr,
	int steps,
	float alpha,
	float beta,
	float gamma,
	float delta,
	float zeta,
	int scaling)
{
	assert(steps >= 0);

	if( scaling < 0 )
	{
		for(int s = 0; s < steps; s++)
		{
			arr[4+s*2] *= 1/zeta;
		}
	
		for(int s = 0; s < steps; s++)
		{
			arr[5+s*2] *= zeta;
		}
	}

	const float coeffs[4] = {delta, gamma, beta, alpha};

	for(int off = 4; off >= 1; off--)
	{
		float *out = arr+off;

		const float c = coeffs[off-1];

		for(int s = 0; s < steps; s++)
		{
			out[0] += c * (out[-1] + out[+1]);

			out += 2;
		}
	}

	if( scaling > 0 )
	{
		for(int s = 0; s < steps; s++)
		{
			arr[0+s*2] *= 1/zeta;
		}

		for(int s = 0; s < steps; s++)
		{
			arr[1+s*2] *= zeta;
		}
	}
}

/**
 * @brief Accelerated PicoBlaze operation.
 *
 * Two pairs (predict and update) of lifting steps and coefficients scaling
 * merged together. This function is accelerated on EdkDSP.
 */
static
void accel_lift_op4s_main_pb_s(
	float *arr,
	int steps,
	float alpha,
	float beta,
	float gamma,
	float delta,
	float zeta,
	int scaling)
{
	assert(steps >= 0);

#ifdef microblaze
	UNUSED(scaling);

	assert( steps <= (BANK_SIZE-4)/2 );

	const float coeffs[11] = {delta, 0.0f, gamma, 0.0f, beta, 0.0f, alpha, 0.0f, zeta, 0.0f, 1/zeta};

	WAL_CHECK( wal_mb2dmem(worker, 0, WAL_BCE_JK_DMEM_A, WAL_BANK_POS(0,0), arr, 2*steps+4) );
	WAL_CHECK( wal_mb2dmem(worker, 0, WAL_BCE_JK_DMEM_B, WAL_BANK_POS(0,0), coeffs, 11) );
	WAL_CHECK( wal_mb2pb(worker, steps) );
	WAL_CHECK( wal_pb2mb(worker, NULL) );
	WAL_CHECK( wal_dmem2mb(worker, 0, WAL_BCE_JK_DMEM_A, WAL_BANK_POS(0,0), arr, 2*steps+4) );
#else
	// fallback
	accel_lift_op4s_main_s(arr, steps, alpha, beta, gamma, delta, zeta, scaling);
#endif
}

static
void accel_lift_op4s_prolog_s(
	float *arr,
	int off,
	int N,
	float alpha,
	float beta,
	float gamma,
	float delta,
	float zeta,
	int scaling)
{
	assert(N-off >= 4);

	if(off)
	{
		// inv-scaling
		if( scaling < 0 )
		{
			// TODO
		}

 		// alpha
		arr[1] += alpha*(arr[0]+arr[2]);
		arr[3] += alpha*(arr[2]+arr[4]);

		// beta
		arr[0] += 2*beta*(arr[1]);
		arr[2] += beta*(arr[1]+arr[3]);
	
		// gamma
		arr[1] += gamma*(arr[0]+arr[2]);
	
		// delta
		arr[0] += 2*delta*(arr[1]);

		// scaling
		if( scaling > 0)
		{
			arr[0] *= zeta;
		}
	}
	else
	{
		// inv-scaling
		if( scaling < 0 )
		{
			arr[0] *= 1/zeta;
			arr[1] *= zeta;
			arr[2] *= 1/zeta;
			arr[3] *= zeta;
		}

		// alpha
		arr[0] += 2*alpha*(arr[1]);
		arr[2] += alpha*(arr[1]+arr[3]);
		
		// beta
		arr[1] += beta*(arr[0]+arr[2]);
	
		// gamma
		arr[0] += 2*gamma*(arr[1]);
	
		// delta
		// není

		// scaling
		// není
	}
}

static
void accel_lift_op4s_epilog_s(
	float *arr,
	int off,
	int N,
	float alpha,
	float beta,
	float gamma,
	float delta,
	float zeta,
	int scaling)
{
	assert(N-off >= 4);

	if( is_even(N-off) )
	{
		// inv-scaling
		if( scaling < 0 )
		{
			// TODO
		}

		// alpha
		// není

		// beta
		arr[N-1] += 2*beta*(arr[N-2]);

		// gamma
		arr[N-2] += gamma*(arr[N-1]+arr[N-3]);

		// delta
		arr[N-1] += 2*delta*(arr[N-2]);
		arr[N-3] += delta*(arr[N-4]+arr[N-2]);

		// scaling
		if( scaling > 0 )
		{
			// FIXME: this is dependend on "off"
			arr[N-4] *= 1/zeta;
			arr[N-3] *= zeta;
			arr[N-2] *= 1/zeta;
			arr[N-1] *= zeta;
		}
	}
	else /* is_odd(N-off) */
	{
		// inv-scaling
		if( scaling < 0 )
		{
			arr[N-1] *= 1/zeta;
		}

		// alpha
		arr[N-1] += 2*alpha*(arr[N-2]);

		// beta
		arr[N-2] += beta*(arr[N-1]+arr[N-3]);

		// gamma
		arr[N-1] += 2*gamma*(arr[N-2]);
		arr[N-3] += gamma*(arr[N-2]+arr[N-4]);

		// delta
		arr[N-2] += delta*(arr[N-1]+arr[N-3]);
		arr[N-4] += delta*(arr[N-5]+arr[N-3]);

		// scaling
		if( scaling > 0 )
		{
			// FIXME: this is dependend on "off"
			arr[N-5] *= 1/zeta;
			arr[N-4] *= zeta;
			arr[N-3] *= 1/zeta;
			arr[N-2] *= zeta;
			arr[N-1] *= 1/zeta;
		}
	}
}

/**
 * @brief Prolog and epilog for N-off < 4.
 */
static
void accel_lift_op4s_short_s(
	float *arr,
	int off,
	int N,
	float alpha,
	float beta,
	float gamma,
	float delta,
	float zeta,
	int scaling)
{
	assert(N-off < 4);

	if(off)
	{
		if( N == 2 )
		{
			// inv-scaling
			if( scaling < 0 )
			{
				// TODO
			}

			// alpha
			arr[1] += 2*alpha*(arr[0]);

			// beta
			arr[0] += 2*beta*(arr[1]);

			// gamma
			arr[1] += 2*gamma*(arr[0]);

			// delta
			arr[0] += 2*delta*(arr[1]);

			// scaling
			if( scaling > 0 )
			{
				arr[0] *= zeta;
				arr[1] *= 1/zeta;
			}
		}
		else
		if( N == 3 )
		{
			// inv-scaling
			if( scaling < 0 )
			{
				// TODO
			}

			// alpha
			arr[1] += alpha*(arr[0]+arr[2]);

			// beta
			arr[0] += 2*beta*(arr[1]);
			arr[2] += 2*beta*(arr[1]);

			// gamma
			arr[1] += gamma*(arr[0]+arr[2]);

			// delta
			arr[0] += 2*delta*(arr[1]);
			arr[2] += 2*delta*(arr[1]);

			// scaling
			if( scaling > 0 )
			{
				arr[0] *= zeta;
				arr[1] *= 1/zeta;
				arr[2] *= zeta;
			}
		}
		else /* N == 4 */
		{
			// inv-scaling
			if( scaling < 0 )
			{
				// TODO
			}

			// alpha
			arr[1] += alpha*(arr[0]+arr[2]);
			arr[3] += 2*alpha*(arr[2]);

			// beta
			arr[0] += 2*beta*(arr[1]);
			arr[2] += beta*(arr[1]+arr[3]);

			// gamma
			arr[1] += gamma*(arr[0]+arr[2]);
			arr[3] += 2*gamma*(arr[2]);

			// delta
			arr[0] += 2*delta*(arr[1]);
			arr[2] += delta*(arr[1]+arr[3]);

			// scaling
			if( scaling > 0 )
			{
				arr[0] *= zeta;
				arr[1] *= 1/zeta;
				arr[2] *= zeta;
				arr[3] *= 1/zeta;
			}
		}
	}
	else /* !off */
	{
		if( N == 2 )
		{
			// inv-scaling
			if( scaling < 0 )
			{
				arr[0] *= 1/zeta;
				arr[1] *= zeta;
			}

			// alpha
			arr[0] += 2*alpha*(arr[1]);

			// beta
			arr[1] += 2*beta*(arr[0]);

			// gamma
			arr[0] += 2*gamma*(arr[1]);

			// delta
			arr[1] += 2*delta*(arr[0]);

			// scaling
			if( scaling > 0 )
			{
				// TODO
			}
		}
		else /* N == 3 */
		{
			// inv-scaling
			if( scaling < 0 )
			{
				arr[0] *= 1/zeta;
				arr[1] *= zeta;
				arr[2] *= 1/zeta;
			}

			// alpha
			arr[0] += 2*alpha*(arr[1]);
			arr[2] += 2*alpha*(arr[1]);

			// beta
			arr[1] += beta*(arr[0]+arr[2]);

			// gamma
			arr[0] += 2*gamma*(arr[1]);
			arr[2] += 2*gamma*(arr[1]);

			// delta
			arr[1] += delta*(arr[0]+arr[2]);

			// scaling
			if( scaling > 0 )
			{
				// TODO
			}
		}
	}
}

static
void accel_lift_op4s_s(
	float *arr,
	int off,
	int len,
	float alpha,
	float beta,
	float gamma,
	float delta,
	float zeta,
	int scaling)
{
	assert(len >= 2);

	assert(0 == off || 1 == off);

	if( len-off < 4 )
	{
		accel_lift_op4s_short_s(arr, off, len, alpha, beta, gamma, delta, zeta, scaling);
	}
	else
	{
		accel_lift_op4s_prolog_s(arr, off, len, alpha, beta, gamma, delta, zeta, scaling);

		if(1 == dwt_util_global_accel_type[0])
		{
			const int max_inner_len = to_even(BANK_SIZE) - 4;
			const int inner_len = to_even(len-off) - 4;
			const int blocks = inner_len / max_inner_len;
			for(int b = 0; b < blocks; b++)
			{
				const int left = off + b * max_inner_len;
				const int steps = max_inner_len/2;

				accel_lift_op4s_main_pb_s(&arr[left], steps, alpha, beta, gamma, delta, zeta, scaling);
			}
			// last block
			if( blocks*max_inner_len < inner_len )
			{
				const int left = off + blocks * max_inner_len;
				const int steps = (off + inner_len - left)/2;

				// TODO: here should be a test if last block should be accelerated on PicoBlaze or rather computed on MicroBlaze
				if( steps > 50 )
					accel_lift_op4s_main_pb_s(&arr[left], steps, alpha, beta, gamma, delta, zeta, scaling);
				else
					accel_lift_op4s_main_s(&arr[left], steps, alpha, beta, gamma, delta, zeta, scaling);
			}
		}
		else if(0 == dwt_util_global_accel_type[0])
		{
			accel_lift_op4s_main_s(arr+off, (to_even(len-off)-4)/2, alpha, beta, gamma, delta, zeta, scaling);
		}
		else if(2 == dwt_util_global_accel_type[0])
		{
			// empty
		}
		else
			abort(); // unsupported value

		accel_lift_op4s_epilog_s(arr, off, len, alpha, beta, gamma, delta, zeta, scaling);
	}
}

void dwt_cdf97_f_ex_stride_s(
	const float *src,
	float *dst_l,
	float *dst_h,
	float *tmp,
	int N,
	int stride)
{
	assert(!( N < 0 || NULL == src || NULL == dst_l || NULL == dst_h || NULL == tmp || 0 == stride ));

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst_l[0] = src[0] * dwt_cdf97_s1_s;
		return;
	}

	// copy src into tmp
	dwt_util_memcpy_stride_s(tmp, sizeof(float), src, stride, N);

	accel_lift_op4s_s(tmp, 1, N, -dwt_cdf97_p1_s, dwt_cdf97_u1_s, -dwt_cdf97_p2_s, dwt_cdf97_u2_s, dwt_cdf97_s1_s, +1);

	// copy tmp into dst
	dwt_util_memcpy_stride_s(dst_l, stride, tmp+0, 2*sizeof(float),  ceil_div2(N));
	dwt_util_memcpy_stride_s(dst_h, stride, tmp+1, 2*sizeof(float), floor_div2(N));
}

void dwt_cdf97_i_ex_d(
	const double *src_l,
	const double *src_h,
	double *dst,
	double *tmp,
	int N)
{
	dwt_cdf97_i_ex_stride_d(
		src_l,
		src_h,
		dst,
		tmp,
		N,
		sizeof(double)
	);
}

void dwt_cdf97_i_ex_s(
	const float *src_l,
	const float *src_h,
	float *dst,
	float *tmp,
	int N)
{
	dwt_cdf97_i_ex_stride_s(
		src_l,
		src_h,
		dst,
		tmp,
		N,
		sizeof(float)
	);
}

void dwt_cdf97_i_ex_stride_d(
	const double *src_l,
	const double *src_h,
	double *dst,
	double *tmp,
	int N,
	int stride)
{
	assert(!( N < 0 || NULL == src_l || NULL == src_h || NULL == dst || NULL == tmp || 0 == stride ));

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst[0] = src_l[0] * dwt_cdf97_s2_d;
		return;
	}

	// copy src into tmp
	dwt_util_memcpy_stride_d(tmp+0, 2*sizeof(double), src_l, stride,  ceil_div2(N));
	dwt_util_memcpy_stride_d(tmp+1, 2*sizeof(double), src_h, stride, floor_div2(N));

	// inverse scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s2_d;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s1_d;

	// backward update 2 + backward predict 2
	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] -= dwt_cdf97_u2_d * (tmp[i-1] + tmp[i+1]);

	tmp[0] -= 2 * dwt_cdf97_u2_d * tmp[1];

	if(is_odd(N))
		tmp[N-1] -= 2 * dwt_cdf97_u2_d * tmp[N-2];
	else
		tmp[N-1] += 2 * dwt_cdf97_p2_d * tmp[N-2];

	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] += dwt_cdf97_p2_d * (tmp[i-1] + tmp[i+1]);

	// backward update 1 + backward predict 1
	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] -= dwt_cdf97_u1_d * (tmp[i-1] + tmp[i+1]);

	tmp[0] -= 2 * dwt_cdf97_u1_d * tmp[1];

	if(is_odd(N))
		tmp[N-1] -= 2 * dwt_cdf97_u1_d * tmp[N-2];
	else
		tmp[N-1] += 2 * dwt_cdf97_p1_d * tmp[N-2];

	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] += dwt_cdf97_p1_d * (tmp[i-1] + tmp[i+1]);

	// copy tmp into dst
	dwt_util_memcpy_stride_d(dst, stride, tmp, sizeof(double), N);
}

void dwt_cdf97_i_ex_stride_s(
	const float *src_l,
	const float *src_h,
	float *dst,
	float *tmp,
	int N,
	int stride)
{
	assert(!( N < 0 || NULL == src_l || NULL == src_h || NULL == dst || NULL == tmp || 0 == stride ));

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst[0] = src_l[0] * dwt_cdf97_s2_s; // FIXME: 1/zeta
		return;
	}

	// copy src into tmp
	dwt_util_memcpy_stride_s(tmp+0, 2*sizeof(float), src_l, stride,  ceil_div2(N));
	dwt_util_memcpy_stride_s(tmp+1, 2*sizeof(float), src_h, stride, floor_div2(N));

	accel_lift_op4s_s(tmp, 0, N, -dwt_cdf97_u2_s, dwt_cdf97_p2_s, -dwt_cdf97_u1_s, dwt_cdf97_p1_s, dwt_cdf97_s1_s, -1);

	// copy tmp into dst
	dwt_util_memcpy_stride_s(dst, stride, tmp, sizeof(float), N);
}

void dwt_zero_padding_f_d(
	double *dst_l,
	double *dst_h,
	int N,
	int N_dst_L,
	int N_dst_H)
{
	dwt_zero_padding_f_stride_d(
		dst_l,
		dst_h,
		N,
		N_dst_L,
		N_dst_H,
		sizeof(double)
	);
}

void dwt_zero_padding_f_s(
	float *dst_l,
	float *dst_h,
	int N,
	int N_dst_L,
	int N_dst_H)
{
	dwt_zero_padding_f_stride_s(
		dst_l,
		dst_h,
		N,
		N_dst_L,
		N_dst_H,
		sizeof(float)
	);
}

void dwt_zero_padding_f_stride_d(
	double *dst_l,
	double *dst_h,
	int N,
	int N_dst_L,
	int N_dst_H,
	int stride)
{
	assert(!( N < 0 || N_dst_L < 0 || N_dst_H < 0 || 0 != ((N_dst_L-N_dst_H)&~1) || NULL == dst_l || NULL == dst_h || 0 == stride ));

	if(N_dst_L || N_dst_H)
	{
		const double zero = 0;

		dwt_util_memcpy_stride_d(addr1_d(dst_l,  ceil_div2(N), stride), stride, &zero, 0, N_dst_L -  ceil_div2(N));
		dwt_util_memcpy_stride_d(addr1_d(dst_h, floor_div2(N), stride), stride, &zero, 0, N_dst_H - floor_div2(N));
	}
}

void dwt_zero_padding_f_stride_s(
	float *dst_l,
	float *dst_h,
	int N,
	int N_dst_L,
	int N_dst_H,
	int stride)
{
	assert(!( N < 0 || N_dst_L < 0 || N_dst_H < 0 || 0 != ((N_dst_L-N_dst_H)&~1) || NULL == dst_l || NULL == dst_h || 0 == stride ));

	if(N_dst_L || N_dst_H)
	{
		const float zero = 0;

		dwt_util_memcpy_stride_s(addr1_s(dst_l,  ceil_div2(N), stride), stride, &zero, 0, N_dst_L -  ceil_div2(N));
		dwt_util_memcpy_stride_s(addr1_s(dst_h, floor_div2(N), stride), stride, &zero, 0, N_dst_H - floor_div2(N));
	}
}

void dwt_zero_padding_i_d(
	double *dst_l,
	int N,
	int N_dst)
{
	dwt_zero_padding_i_stride_d(
		dst_l,
		N,
		N_dst,
		sizeof(double)
	);
}

void dwt_zero_padding_i_s(
	float *dst_l,
	int N,
	int N_dst)
{
	dwt_zero_padding_i_stride_s(
		dst_l,
		N,
		N_dst,
		sizeof(float)
	);
}

void dwt_zero_padding_i_stride_d(
	double *dst_l,
	int N,
	int N_dst,
	int stride)
{
	assert(!( N < 0 || N_dst < 0 || NULL == dst_l || 0 == stride ));

	const double zero = 0;

	dwt_util_memcpy_stride_d(
		addr1_d(dst_l, N, stride),
		stride,
		&zero,
		0,
		N_dst - N);
}

void dwt_zero_padding_i_stride_s(
	float *dst_l,
	int N,
	int N_dst,
	int stride)
{
	assert(!( N < 0 || N_dst < 0 || NULL == dst_l || 0 == stride ));

	const float zero = 0;

	dwt_util_memcpy_stride_s(
		addr1_s(dst_l, N, stride),
		stride,
		&zero,
		0,
		N_dst - N);
}

#ifdef microblaze
const unsigned int DWT_OP_LIFT4SA = WAL_PBID_P0;
const unsigned int DWT_OP_LIFT4SB = WAL_PBID_P1;
#else
const unsigned int DWT_OP_LIFT4SA = 0;
const unsigned int DWT_OP_LIFT4SB = 1;
#endif

// TODO: export due to usage of stand-alone 1D transform
void dwt_util_switch_op(
	unsigned int pbid)
{
#ifdef microblaze
	WAL_CHECK( wal_mb2pb(worker, 0) );

	WAL_CHECK( wal_bce_jk_sync_operation(worker) );

	WAL_CHECK( wal_reset_worker(worker) );

	WAL_CHECK( wal_start_operation(worker, pbid) );
#else
	UNUSED(pbid);
#endif
}

void dwt_cdf97_2f_d(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y,
	int size_i_big_x,
	int size_i_big_y,
	int *j_max_ptr,
	int decompose_one,
	int zero_padding)
{
	const int size_o_big_min = min(size_o_big_x,size_o_big_y);
	const int size_o_big_max = max(size_o_big_x,size_o_big_y);

	double temp[size_o_big_max];
	if(NULL == temp)
		abort();

	int j = 0;

	const int j_limit = ceil_log2(decompose_one?size_o_big_max:size_o_big_min);

	if( *j_max_ptr < 0 || *j_max_ptr > j_limit )
		*j_max_ptr = j_limit;

	for(;;)
	{
		if( *j_max_ptr == j )
			break;

		const int size_o_src_x = ceil_div_pow2(size_o_big_x, j  );
		const int size_o_src_y = ceil_div_pow2(size_o_big_y, j  );
		const int size_o_dst_x = ceil_div_pow2(size_o_big_x, j+1);
		const int size_o_dst_y = ceil_div_pow2(size_o_big_y, j+1);
		const int size_i_src_x = ceil_div_pow2(size_i_big_x, j  );
		const int size_i_src_y = ceil_div_pow2(size_i_big_y, j  );

		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_src_y, omp_get_num_threads()))
		for(int y = 0; y < size_o_src_y; y++)
			dwt_cdf97_f_ex_stride_d(
				addr2_d(ptr,y,0,stride_x,stride_y),
				addr2_d(ptr,y,0,stride_x,stride_y),
				addr2_d(ptr,y,size_o_dst_x,stride_x,stride_y),
				temp,
				size_i_src_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_src_x; x++)
			dwt_cdf97_f_ex_stride_d(
				addr2_d(ptr,0,x,stride_x,stride_y),
				addr2_d(ptr,0,x,stride_x,stride_y),
				addr2_d(ptr,size_o_dst_y,x,stride_x,stride_y),
				temp,
				size_i_src_y,
				stride_x);

		if(zero_padding)
		{
			#pragma omp parallel for schedule(static, ceil_div(size_o_src_y, omp_get_num_threads()))
			for(int y = 0; y < size_o_src_y; y++)
				dwt_zero_padding_f_stride_d(
					addr2_d(ptr,y,0,stride_x,stride_y),
					addr2_d(ptr,y,size_o_dst_x,stride_x,stride_y),
					size_i_src_x,
					size_o_dst_x,
					size_o_src_x-size_o_dst_x,
					stride_y);
			#pragma omp parallel for schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
			for(int x = 0; x < size_o_src_x; x++)
				dwt_zero_padding_f_stride_d(
					addr2_d(ptr,0,x,stride_x,stride_y),
					addr2_d(ptr,size_o_dst_y,x,stride_x,stride_y),
					size_i_src_y,
					size_o_dst_y,
					size_o_src_y-size_o_dst_y,
					stride_x);
		}

		j++;
	}
}

void dwt_cdf97_2f_s(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y,
	int size_i_big_x,
	int size_i_big_y,
	int *j_max_ptr,
	int decompose_one,
	int zero_padding)
{
	dwt_util_switch_op(DWT_OP_LIFT4SA);

	const int size_o_big_min = min(size_o_big_x,size_o_big_y);
	const int size_o_big_max = max(size_o_big_x,size_o_big_y);

	float temp[size_o_big_max];
	if(NULL == temp)
		abort();

	int j = 0;

	const int j_limit = ceil_log2(decompose_one?size_o_big_max:size_o_big_min);

	if( *j_max_ptr < 0 || *j_max_ptr > j_limit )
		*j_max_ptr = j_limit;

	for(;;)
	{
		if( *j_max_ptr == j )
			break;

		const int size_o_src_x = ceil_div_pow2(size_o_big_x, j  );
		const int size_o_src_y = ceil_div_pow2(size_o_big_y, j  );
		const int size_o_dst_x = ceil_div_pow2(size_o_big_x, j+1);
		const int size_o_dst_y = ceil_div_pow2(size_o_big_y, j+1);
		const int size_i_src_x = ceil_div_pow2(size_i_big_x, j  );
		const int size_i_src_y = ceil_div_pow2(size_i_big_y, j  );

		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_src_y, omp_get_num_threads()))
		for(int y = 0; y < size_o_src_y; y++)
			dwt_cdf97_f_ex_stride_s(
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,size_o_dst_x,stride_x,stride_y),
				temp,
				size_i_src_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_src_x; x++)
			dwt_cdf97_f_ex_stride_s(
				addr2_s(ptr,0,x,stride_x,stride_y),
				addr2_s(ptr,0,x,stride_x,stride_y),
				addr2_s(ptr,size_o_dst_y,x,stride_x,stride_y),
				temp,
				size_i_src_y,
				stride_x);

		if(zero_padding)
		{
			#pragma omp parallel for schedule(static, ceil_div(size_o_src_y, omp_get_num_threads()))
			for(int y = 0; y < size_o_src_y; y++)
				dwt_zero_padding_f_stride_s(
					addr2_s(ptr,y,0,stride_x,stride_y),
					addr2_s(ptr,y,size_o_dst_x,stride_x,stride_y),
					size_i_src_x,
					size_o_dst_x,
					size_o_src_x-size_o_dst_x,
					stride_y);
			#pragma omp parallel for schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
			for(int x = 0; x < size_o_src_x; x++)
				dwt_zero_padding_f_stride_s(
					addr2_s(ptr,0,x,stride_x,stride_y),
					addr2_s(ptr,size_o_dst_y,x,stride_x,stride_y),
					size_i_src_y,
					size_o_dst_y,
					size_o_src_y-size_o_dst_y,
					stride_x);
		}

		j++;
	}
}

void dwt_cdf97_2i_d(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y,
	int size_i_big_x,
	int size_i_big_y,
	int j_max,
	int decompose_one,
	int zero_padding)
{
	const int size_o_big_min = min(size_o_big_x,size_o_big_y);
	const int size_o_big_max = max(size_o_big_x,size_o_big_y);

	double temp[size_o_big_max];
	if(NULL == temp)
		abort();

	int j = ceil_log2(decompose_one?size_o_big_max:size_o_big_min);

	if( j_max >= 0 && j_max < j )
		j = j_max;

	for(;;)
	{
		if(0 == j)
			break;

		const int size_o_src_x = ceil_div_pow2(size_o_big_x, j  );
		const int size_o_src_y = ceil_div_pow2(size_o_big_y, j  );
		const int size_o_dst_x = ceil_div_pow2(size_o_big_x, j-1);
		const int size_o_dst_y = ceil_div_pow2(size_o_big_y, j-1);
		const int size_i_dst_x = ceil_div_pow2(size_i_big_x, j-1);
		const int size_i_dst_y = ceil_div_pow2(size_i_big_y, j-1);

		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_dst_y, omp_get_num_threads()))
		for(int y = 0; y < size_o_dst_y; y++)
			dwt_cdf97_i_ex_stride_d(
				addr2_d(ptr,y,0,stride_x,stride_y),
				addr2_d(ptr,y,size_o_src_x,stride_x,stride_y),
				addr2_d(ptr,y,0,stride_x,stride_y),
				temp,
				size_i_dst_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_dst_x; x++)
			dwt_cdf97_i_ex_stride_d(
				addr2_d(ptr,0,x,stride_x,stride_y),
				addr2_d(ptr,size_o_src_y,x,stride_x,stride_y),
				addr2_d(ptr,0,x,stride_x,stride_y),
				temp,
				size_i_dst_y,
				stride_x);

		if(zero_padding)
		{
			#pragma omp parallel for schedule(static, ceil_div(size_o_dst_y, omp_get_num_threads()))
			for(int y = 0; y < size_o_dst_y; y++)
				dwt_zero_padding_i_stride_d(
					addr2_d(ptr,y,0,stride_x,stride_y),
					size_i_dst_x,
					size_o_dst_x,
					stride_y);
			#pragma omp parallel for schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
			for(int x = 0; x < size_o_dst_x; x++)
				dwt_zero_padding_i_stride_d(
					addr2_d(ptr,0,x,stride_x,stride_y),
					size_i_dst_y,
					size_o_dst_y,
					stride_x);
		}

		j--;
	}
}

void dwt_cdf97_2i_s(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y,
	int size_i_big_x,
	int size_i_big_y,
	int j_max,
	int decompose_one,
	int zero_padding)
{
	dwt_util_switch_op(DWT_OP_LIFT4SB);

	const int size_o_big_min = min(size_o_big_x,size_o_big_y);
	const int size_o_big_max = max(size_o_big_x,size_o_big_y);

	float temp[size_o_big_max];
	if(NULL == temp)
		abort();

	int j = ceil_log2(decompose_one?size_o_big_max:size_o_big_min);

	if( j_max >= 0 && j_max < j )
		j = j_max;

	for(;;)
	{
		if(0 == j)
			break;

		const int size_o_src_x = ceil_div_pow2(size_o_big_x, j  );
		const int size_o_src_y = ceil_div_pow2(size_o_big_y, j  );
		const int size_o_dst_x = ceil_div_pow2(size_o_big_x, j-1);
		const int size_o_dst_y = ceil_div_pow2(size_o_big_y, j-1);
		const int size_i_dst_x = ceil_div_pow2(size_i_big_x, j-1);
		const int size_i_dst_y = ceil_div_pow2(size_i_big_y, j-1);

		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_dst_y, omp_get_num_threads()))
		for(int y = 0; y < size_o_dst_y; y++)
			dwt_cdf97_i_ex_stride_s(
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,size_o_src_x,stride_x,stride_y),
				addr2_s(ptr,y,0,stride_x,stride_y),
				temp,
				size_i_dst_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_dst_x; x++)
			dwt_cdf97_i_ex_stride_s(
				addr2_s(ptr,0,x,stride_x,stride_y),
				addr2_s(ptr,size_o_src_y,x,stride_x,stride_y),
				addr2_s(ptr,0,x,stride_x,stride_y),
				temp,
				size_i_dst_y,
				stride_x);

		if(zero_padding)
		{
			#pragma omp parallel for schedule(static, ceil_div(size_o_dst_y, omp_get_num_threads()))
			for(int y = 0; y < size_o_dst_y; y++)
				dwt_zero_padding_i_stride_s(
					addr2_s(ptr,y,0,stride_x,stride_y),
					size_i_dst_x,
					size_o_dst_x,
					stride_y);
			#pragma omp parallel for schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
			for(int x = 0; x < size_o_dst_x; x++)
				dwt_zero_padding_i_stride_s(
					addr2_s(ptr,0,x,stride_x,stride_y),
					size_i_dst_y,
					size_o_dst_y,
					stride_x);
		}

		j--;
	}
}

int dwt_util_clock_autoselect()
{
#ifdef ENABLE_TIME_CLOCK_GETTIME
	return DWT_TIME_CLOCK_GETTIME;
#endif
#ifdef ENABLE_TIME_TIMES
	return DWT_TIME_TIMES;
#endif
#ifdef ENABLE_TIME_CLOCK
	return DWT_TIME_CLOCK;
#endif
#ifdef ENABLE_TIME_GETRUSAGE
	return DWT_TIME_GETRUSAGE;
#endif
	// fallback
	return DWT_TIME_AUTOSELECT;
}

dwt_clock_t dwt_util_get_frequency(
	int type)
{
	if(DWT_TIME_AUTOSELECT == type)
		type = dwt_util_clock_autoselect();

	dwt_clock_t return_freq;

	switch(type)
	{
		case DWT_TIME_CLOCK_GETTIME:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME
			return_freq = (dwt_clock_t)1000000000;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK:
		{
#ifdef ENABLE_TIME_CLOCK
			return_freq = (dwt_clock_t)CLOCKS_PER_SEC;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_TIMES:
		{
#ifdef ENABLE_TIME_TIMES
			long ticks_per_sec = sysconf(_SC_CLK_TCK);

			return_freq = (dwt_clock_t)ticks_per_sec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE:
		{
#ifdef ENABLE_TIME_GETRUSAGE
			return_freq = (dwt_clock_t)1000000000;
#else
			abort();
#endif
		}
		break;
		default:
			abort();
	}

	return return_freq;
}

dwt_clock_t dwt_util_get_clock(
	int type)
{
	if(DWT_TIME_AUTOSELECT == type)
		type = dwt_util_clock_autoselect();

	dwt_clock_t return_time;

	switch(type)
	{
		case DWT_TIME_CLOCK_GETTIME:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME
			clockid_t clk_id = /*CLOCK_THREAD_CPUTIME_ID*/ /*CLOCK_PROCESS_CPUTIME_ID*/ CLOCK_REALTIME; // FIXME

			dwt_clock_t time;

			struct timespec ts;

			if(clock_gettime(clk_id, &ts))
				abort();

			time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;

			return_time = (dwt_clock_t)time;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK:
		{
#ifdef ENABLE_TIME_CLOCK
			clock_t time;

			time = clock();

			return_time = (dwt_clock_t)time;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_TIMES:
		{
#ifdef ENABLE_TIME_TIMES
			clock_t time;

			struct tms tms_i;

			if( (clock_t)-1 == times(&tms_i) )
				abort();
			time = tms_i.tms_utime;

			return_time = (dwt_clock_t)time;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE:
		{
#ifdef ENABLE_TIME_GETRUSAGE
			int who = RUSAGE_SELF /*RUSAGE_THREAD*/; // FIXME

			dwt_clock_t time;

			struct timeval tv;
			struct rusage rusage_i;
			struct timespec ts;

			if( -1 == getrusage(who, &rusage_i) )
				abort();
			tv = rusage_i.ru_utime;
			TIMEVAL_TO_TIMESPEC(&tv, &ts);

			time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;

			return_time = (dwt_clock_t)time;
#else
			abort();
#endif
		}
		break;
		default:
			abort();
	}

	return return_time;
}

int dwt_util_get_thread_num()
{
#ifdef _OPENMP
	return omp_get_thread_num();
#else
	return 0;
#endif
}

int dwt_util_get_max_threads()
{
#ifdef _OPENMP
	return omp_get_max_threads();
#else
	return 1;
#endif
}

void dwt_util_set_num_threads(
	int num_threads)
{
#ifdef _OPENMP
	omp_set_num_threads(num_threads);
#else
	UNUSED(num_threads);
#endif
}

int dwt_util_get_num_threads()
{
#ifdef _OPENMP
	return omp_get_num_threads();
#else
	return 1;
#endif
}

void dwt_util_init()
{
#ifdef microblaze
	WAL_CHECK( wal_init_worker(worker) );

	WAL_CHECK( wal_set_firmware(worker, /*WAL_PBID_P0*/DWT_OP_LIFT4SA, fw_fp01_lift4sa, -1) );

	WAL_CHECK( wal_set_firmware(worker, /*WAL_PBID_P1*/DWT_OP_LIFT4SB, fw_fp01_lift4sb, -1) );

	WAL_CHECK( wal_reset_worker(worker) );

	WAL_CHECK( wal_start_operation(worker, WAL_PBID_P0) );

	dwt_util_set_accel(1);
#endif
}

void dwt_util_finish()
{
#ifdef microblaze
	WAL_CHECK( wal_mb2pb(worker, 0) );

	WAL_CHECK( wal_bce_jk_sync_operation(worker) );

	WAL_CHECK( wal_done_worker(worker) );
#endif
}

int dwt_util_save_to_pgm_s(
	const char *filename,
	float max_value,
	void *ptr,
	int stride_x,
	int stride_y,
	int size_i_big_x,
	int size_i_big_y)
{
	const int target_max_value = 255;

	FILE *file = fopen(filename, "w");
	if(NULL == file)
		return 1;

	fprintf(file, "P2\n%i %i\n%i\n", size_i_big_x, size_i_big_y, target_max_value);

	for(int y = 0; y < size_i_big_y; y++)
		for(int x = 0; x < size_i_big_x; x++)
		{
			const float px = *addr2_s(ptr, y, x, stride_x, stride_y);
			if( fprintf(file, "%i\n", (int)(target_max_value*px/max_value)) < 0)
			{
				fclose(file);
				return 1;
			}
		}

	fclose(file);

	return 0;
}

void dwt_util_set_accel(
	int accel_type)
{
	dwt_util_global_accel_type[0] = accel_type;
}
