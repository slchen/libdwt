/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Fast wavelet transform implemented via lifting scheme.
 */
#include "libdwt.h"

#define DEBUG_VERBOSE
#define USE_DMA

#define STRING(x) #x

#ifdef NDEBUG
	/* Release build */
	#undef DEBUG
	#define FUNC_BEGIN
	#define FUNC_END
#else
	/* Debug build */
	#define DEBUG
	#ifdef DEBUG_VERBOSE
		#define FUNC_BEGIN printf("DEBUG: %s ENTRY\n", __FUNCTION__);
		#define FUNC_END printf("DEBUG: %s EXIT\n", __FUNCTION__);
	#else
		#define FUNC_BEGIN
		#define FUNC_END
	#endif
#endif

/** UTIA EdkDSP specific code */
#ifdef microblaze
	#define WAL_NATIVE_DMA
	#include <wal.h>
	#include <wal_bce_dma.h>
	#include <bce_dma_config.h>

	WAL_REGISTER_WORKER(worker0, BCE_DMA_GENERIC_4D, bce_dma_cfgtable, 0, 1, 0);
	WAL_REGISTER_WORKER(worker1, BCE_DMA_GENERIC_4D, bce_dma_cfgtable, 1, 1, 0);
	WAL_REGISTER_WORKER(worker2, BCE_DMA_GENERIC_4D, bce_dma_cfgtable, 2, 1, 0);
	WAL_REGISTER_WORKER(worker3, BCE_DMA_GENERIC_4D, bce_dma_cfgtable, 3, 1, 0);

	// FIXME: is woraround for global variables required here? var[2] = { ... };
	const int dwt_util_global_total_workers = BCE_DMA_CFGTABLE_NUM_ITEMS; ///< total number of workers available

	wal_worker_t *worker[BCE_DMA_CFGTABLE_NUM_ITEMS] = {
		&worker0_data_structure,
#if BCE_DMA_CFGTABLE_NUM_ITEMS > 1
		&worker1_data_structure,
#endif
#if BCE_DMA_CFGTABLE_NUM_ITEMS > 2
		&worker2_data_structure,
#endif
#if BCE_DMA_CFGTABLE_NUM_ITEMS > 3
		&worker3_data_structure,
#endif
	};

	#include <stddef.h> // ptrdiff_t
	int dwt_util_global_active_workers = BCE_DMA_CFGTABLE_NUM_ITEMS; ///< how many workers use for computation (can be less than total number of workers)
	int dwt_util_global_temp_step; ///< in elements; offset in temp[] is gigen by worker_id * dwt_util_global_temp_step
	ptrdiff_t dwt_util_global_data_step; ///< in bytes; offset in src[] and dst[] is given by worker_id * dwt_util_global_data_step

	#include "firmware/fw_fp01_lift4sa.h"
	#include "firmware/fw_fp01_lift4sb.h"

	#define BANK_SIZE 1024

	// FIXME: is this macro still correct?
	#define WAL_BANK_POS(bank, off) ( (bank)*BANK_SIZE + (off) )

	#define WAL_DMA_MASK(ch) ( 1<<(ch) )

	#ifdef NDEBUG
		/* Release build */
		#define WAL_CHECK(expr) (expr)
	#else
		/* Debug build */
		#ifdef DEBUG_VERBOSE
			#define WAL_CHECK(expr) ( printf("DEBUG: %s = ", STRING(expr)), wal_abort(expr) )
		#else
			#define WAL_CHECK(expr) ( wal_abort(expr) )
		#endif
	#endif
#endif

#ifndef BANK_SIZE
	#define BANK_SIZE 4096
#endif

/** disable timers when using Par4All tool */
#if !defined(P4A)
	#define USE_TIME_CLOCK
	#define USE_TIME_CLOCK_GETTIME
	#define USE_TIME_CLOCK_GETTIME_REALTIME
	#define USE_TIME_CLOCK_GETTIME_MONOTONIC
	#define USE_TIME_CLOCK_GETTIME_MONOTONIC_RAW
	#define USE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID
	#define USE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID
	#define USE_TIME_TIMES
	#define USE_TIME_IOCTL_RTC
	#define USE_TIME_GETTIMEOFDAY
	#define USE_TIME_GETRUSAGE
	#define USE_TIME_GETRUSAGE_SELF
	#define USE_TIME_GETRUSAGE_CHILDREN
	#define USE_TIME_GETRUSAGE_THREAD
#endif

#ifdef DEBUG
	// BUG: workaround due to gcc?
	#warning Usign ugly GCC workaround!
	#undef USE_TIME_CLOCK
	#undef USE_TIME_TIMES
	// FIXME: is this now needed?
	//#define UGLY_GCC_WORKAROUND
#endif

/** include LINUX_VERSION_CODE and KERNEL_VERSION macros */
#if defined(__linux) && !defined(microblaze)
	#include <linux/version.h>
#endif

/** define HAVE_TIME_* macros when corresponding timers are available */
#if defined(_GNU_SOURCE) || defined(_ISOC99_SOURCE) || defined(_POSIX_C_SOURCE)
	#define HAVE_TIME_CLOCK
#endif

#if _POSIX_C_SOURCE >= 199309L || _XOPEN_SOURCE >= 500
	#define HAVE_TIME_CLOCK_GETTIME

	#ifdef _POSIX_C_SOURCE
		#include <unistd.h> // _POSIX_TIMERS, _POSIX_MONOTONIC_CLOCK, _POSIX_CPUTIME, _POSIX_THREAD_CPUTIME

		#ifdef _POSIX_TIMERS
			#define HAVE_TIME_CLOCK_GETTIME_REALTIME

			#ifdef _POSIX_MONOTONIC_CLOCK
				#define HAVE_TIME_CLOCK_GETTIME_MONOTONIC
			#endif

			#if defined(__linux) && !defined(microblaze)
				#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
					// FIXME: probably, at least some glibc version is needed for HAVE_TIME_CLOCK_GETTIME_MONOTONIC_RAW
				#endif
			#endif

			#ifdef _POSIX_CPUTIME
				#define HAVE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID
			#endif

			#ifdef _POSIX_THREAD_CPUTIME
				#define HAVE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID
			#endif
		#endif
	#else
		#define HAVE_TIME_CLOCK_GETTIME_REALTIME
		#define HAVE_TIME_CLOCK_GETTIME_MONOTONIC
		#define HAVE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID
		#define HAVE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID
	#endif
#endif

#if defined(_GNU_SOURCE) || defined(_SVID_SOURCE) || defined(_BSD_SOURCE) || defined(_POSIX_C_SOURCE)
	#define HAVE_TIME_TIMES
#endif

#if defined(__linux) && !defined(microblaze)
	#define HAVE_TIME_IOCTL_RTC
#endif

#if defined(_GNU_SOURCE) || defined(_SVID_SOURCE) || defined(_BSD_SOURCE) || defined(_POSIX_C_SOURCE)
	#define HAVE_TIME_GETTIMEOFDAY
#endif

#if defined(_GNU_SOURCE) || defined(_SVID_SOURCE) || defined(_BSD_SOURCE) || defined(_POSIX_C_SOURCE)
	#define HAVE_TIME_GETRUSAGE
	#define HAVE_TIME_GETRUSAGE_SELF
	#define HAVE_TIME_GETRUSAGE_CHILDREN

	#if defined(__linux) && !defined(microblaze)
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
			#define HAVE_TIME_GETRUSAGE_THREAD
		#endif
	#endif
#endif

/** define ENABLE_TIME_* macros when they are available and intended for use */
#if defined(USE_TIME_CLOCK) && defined(HAVE_TIME_CLOCK)
	#define ENABLE_TIME_CLOCK
#endif

#if defined(USE_TIME_CLOCK_GETTIME) && defined(HAVE_TIME_CLOCK_GETTIME)
	#define ENABLE_TIME_CLOCK_GETTIME
#endif

#if defined(USE_TIME_CLOCK_GETTIME_REALTIME) && defined(HAVE_TIME_CLOCK_GETTIME_REALTIME)
	#define ENABLE_TIME_CLOCK_GETTIME_REALTIME
#endif

#if defined(USE_TIME_CLOCK_GETTIME_MONOTONIC) && defined(HAVE_TIME_CLOCK_GETTIME_MONOTONIC)
	#define ENABLE_TIME_CLOCK_GETTIME_MONOTONIC
#endif

#if defined(USE_TIME_CLOCK_GETTIME_MONOTONIC_RAW) && defined(HAVE_TIME_CLOCK_GETTIME_MONOTONIC_RAW)
	#define ENABLE_TIME_CLOCK_GETTIME_MONOTONIC_RAW
#endif

#if defined(USE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID) && defined(HAVE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID)
	#define ENABLE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID
#endif

#if defined(USE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID) && defined(HAVE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID)
	#define ENABLE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID
#endif

#if defined(USE_TIME_TIMES) && defined(HAVE_TIME_TIMES)
	#define ENABLE_TIME_TIMES
#endif

#if defined(USE_TIME_GETRUSAGE) && defined(HAVE_TIME_GETRUSAGE)
	#define ENABLE_TIME_GETRUSAGE
#endif

#if defined(USE_TIME_IOCTL_RTC) && defined(HAVE_TIME_IOCTL_RTC)
	#define ENABLE_TIME_IOCTL_RTC
#endif

#if defined(USE_TIME_GETTIMEOFDAY) && defined(HAVE_TIME_GETTIMEOFDAY)
	#define ENABLE_TIME_GETTIMEOFDAY
#endif

#if defined(USE_TIME_GETRUSAGE_SELF) && defined(HAVE_TIME_GETRUSAGE_SELF)
	#define ENABLE_TIME_GETRUSAGE_SELF
#endif

#if defined(USE_TIME_GETRUSAGE_CHILDREN) && defined(HAVE_TIME_GETRUSAGE_CHILDREN)
	#define ENABLE_TIME_GETRUSAGE_CHILDREN
#endif

#if defined(USE_TIME_GETRUSAGE_THREAD) && defined(HAVE_TIME_GETRUSAGE_THREAD)
	#define ENABLE_TIME_GETRUSAGE_THREAD
#endif

/** include necessary headers for selected timers */
#if defined(ENABLE_TIME_CLOCK_GETTIME) \
	|| defined(ENABLE_TIME_CLOCK_GETTIME_REALTIME) \
	|| defined(ENABLE_TIME_CLOCK_GETTIME_MONOTONIC) \
	|| defined(ENABLE_TIME_CLOCK_GETTIME_MONOTONIC_RAW) \
	|| defined(ENABLE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID) \
	|| defined(ENABLE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID)

	// NOTE: -lrt
	#include <time.h>  // struct timespec, clock_gettime, CLOCK_REALTIME, CLOCK_MONOTONIC, CLOCK_PROCESS_CPUTIME_ID, CLOCK_THREAD_CPUTIME_ID
#endif

#ifdef ENABLE_TIME_CLOCK
	#include <time.h> // clock, CLOCKS_PER_SEC
#endif

#ifdef ENABLE_TIME_TIMES
	#include <sys/times.h> // struct tms, times
	#include <unistd.h> // sysconf, _SC_CLK_TCK
#endif

#ifdef ENABLE_TIME_IOCTL_RTC
	#include <sys/ioctl.h> // ioctl
	#include <linux/rtc.h> // struct rtc_time, RTC_RD_TIME
	#include <fcntl.h> // open, O_NONBLOCK, O_RDONLY
	#include <unistd.h> // close
	#include <time.h> // struct tm, mktime
#endif

#ifdef ENABLE_TIME_GETTIMEOFDAY
	#include <sys/time.h> // struct timeval, gettimeofday
#endif

#if defined(ENABLE_TIME_GETRUSAGE) \
	|| defined(ENABLE_TIME_GETRUSAGE_SELF) \
	|| defined(ENABLE_TIME_GETRUSAGE_CHILDREN) \
	|| defined(ENABLE_TIME_GETRUSAGE_THREAD)

	#include <time.h> // struct timespec
	#include <unistd.h>
	#include <sys/resource.h> // getrusage, RUSAGE_SELF, struct rusage
	#include <sys/time.h> // struct timeval, TIMEVAL_TO_TIMESPEC
#endif

/** other headers */
#include <assert.h> // assert
#include <stddef.h> // NULL, size_t
#include <stdlib.h> // abort, malloc, free
#include <limits.h> // CHAR_BIT
// NOTE: -lm
#include <math.h> // fabs, fabsf, isnan, isinf
#include <stdio.h> // FILE, fopen, fprintf, fclose
#include <string.h> // memcpy
#include <stdarg.h> // va_start, va_end

/** OpenMP header when used */
#ifdef _OPENMP
	#include <omp.h>
#endif

/** UNUSED macro */
#define UNUSED(expr) do { (void)(expr); } while (0)

/** this PACKAGE_STRING macro must be defined via compiler's command line */
#ifndef PACKAGE_STRING
	#error PACKAGE_STRING is not defined
#endif

/** quoting macros */
#define QUOTE(x) STRING(x)

// FIXME: here, take into account padding at borders when data are not aligned at 64-bits
// FIXME: s/$/_s$/, s/step/offset/
/** Calc offset in temp[] array for current worker. */
static
float *calc_temp_step(
	float *addr,	///< pointer to array assigned to worker 0
	int worker_id	///< identifier of current worker
	)
{
#ifdef microblaze
	return addr + (dwt_util_global_temp_step * worker_id);
#else
	UNUSED(worker_id);
	return addr;
#endif
}

// FIXME: s/$/_s$/, s/step/offset/
/** Calc offset in src[] or dst[] array for current worker. */
static
float *calc_data_step(
	float *addr,	///< pointer to array assigned to worker 0
	int worker_id	///< identifier of current worker
       )
{
#ifdef microblaze
	return (float *)( (intptr_t)addr + (dwt_util_global_data_step * worker_id) );
#else
	UNUSED(worker_id);
	return addr;
#endif
}

// FIXME: suffix _s, s/step/offset/
/** Calc offset in src[] or dst[] array for current worker. */
static
const float *calc_data_step_const(
	const float *addr,	///< pointer to array assigned to worker 0
	int worker_id	///< identifier of current worker
       )
{
#ifdef microblaze
	return (const float *)( (intptr_t)addr + (dwt_util_global_data_step * worker_id) );
#else
	UNUSED(worker_id);
	return addr;
#endif
}

#ifdef microblaze
static inline
void flush_cache(
	intptr_t addr,
	size_t size)
{
	const size_t dcache_line_len = 4;

	// FIXME: should this function before and after "wct" disable and enable dcache like in kernel here -- http://lxr.linux.no/linux+v3.6.6/arch/microblaze/kernel/cpu/cache.c
	// FIXME: this code is weird, either addition of 1 in for-loop or multiplication by dcache_line_len should be removed
	// FIXME: <= would make more sense
	for(size_t i = 0; i < size; i += dcache_line_len)
	{
#ifdef UGLY_GCC_WORKAROUND
		;
#else /* UGLY_GCC_WORKAROUND */
		__asm volatile (
			"wdc %0, r0;"
			:
			: "r" (addr + i * dcache_line_len)
			: "memory"
		);
#endif /* UGLY_GCC_WORKAROUND */
	}
}
#endif

#ifdef microblaze
void wal_abort(int res)
{
	switch(res)
	{
		case WAL_RES_OK:
		#ifdef DEBUG_VERBOSE
			printf("WAL_RES_OK (all is OK)\n");
		#endif
			return;
			break;
		case WAL_RES_WNULL:
			printf("WAL_RES_WNULL (argument is a NULL)\n");
			return;
			break;
		case WAL_RES_ERR:
			printf("WAL_RES_ERR (generic error)\n");
			break;
		case WAL_RES_ENOINIT:
			printf("WAL_RES_ENOINIT (not initiated)\n");
			break;
		case WAL_RES_ENULL:
			printf("WAL_RES_ENULL (null pointer)\n");
			break;
		case WAL_RES_ERUNNING:
			printf("WAL_RES_ERUNNING (worker is running)\n");
			break;
		case WAL_RES_ERANGE:
			printf("WAL_RES_ERANGE (index/value is out of range)\n");
			break;
		default:
			printf("(unknown error)\n");
	}

	dwt_util_abort();
}
#endif

const char *dwt_util_version()
{
	return QUOTE(PACKAGE_STRING);
}

/**
 * @brief Global variable indicating used acceleration type.
 * @note Array of size of 2 is workaround due to GCC 3.4.1 bug.
 */
// FIXME: is this workaround still needed?
int dwt_util_global_accel_type[2] = {0};

/**
 * @{
 * @brief CDF 9/7 lifting scheme constants
 * These constants are found in S. Mallat. A Wavelet Tour of Signal Processing: The Sparse Way (Third Edition). 3rd edition, 2009 on page 370.
 */
static const int    dwt_cdf97_k_s  =    2;
static const float  dwt_cdf97_p1_s =    1.58613434342059;
static const float  dwt_cdf97_u1_s =   -0.0529801185729;
static const float  dwt_cdf97_p2_s =   -0.8829110755309;
static const float  dwt_cdf97_u2_s =    0.4435068520439;
static const float  dwt_cdf97_s1_s =    1.1496043988602;
static const float  dwt_cdf97_s2_s =  1/1.1496043988602; // FIXME: unnecessary

static const int    dwt_cdf97_k_d  =    2;
static const double dwt_cdf97_p1_d =    1.58613434342059;
static const double dwt_cdf97_u1_d =   -0.0529801185729;
static const double dwt_cdf97_p2_d =   -0.8829110755309;
static const double dwt_cdf97_u2_d =    0.4435068520439;
static const double dwt_cdf97_s1_d =    1.1496043988602;
static const double dwt_cdf97_s2_d =  1/1.1496043988602; // FIXME: unnecessary
/**@}*/

/**
 * @{
 * @brief CDF 5/3 lifting scheme constants
 * These constants are found in S. Mallat. A Wavelet Tour of Signal Processing: The Sparse Way (Third Edition). 3rd edition, 2009 on page 369.
 */
static const int    dwt_cdf53_k_s  =    1;
static const float  dwt_cdf53_p1_s =    0.5;
static const float  dwt_cdf53_u1_s =    0.25;
static const float  dwt_cdf53_s1_s =    1.41421356237309504880;
static const float  dwt_cdf53_s2_s =    0.70710678118654752440; // FIXME: unnecessary

static const int    dwt_cdf53_k_d  =    1;
static const double dwt_cdf53_p1_d =    0.5;
static const double dwt_cdf53_u1_d =    0.25;
static const double dwt_cdf53_s1_d =    1.41421356237309504880;
static const double dwt_cdf53_s2_d =    0.70710678118654752440; // FIXME: unnecessary
/**@}*/

/**
 * @brief Power of two using greater or equal to x, i.e. 2^(ceil(log_2(x)).
 */
static
int pow2_ceil_log2(
	int x)
{
	assert( x > 0 );

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

/**
 * @brief returns 1 if x is odd, 0 otherwise; works also for negative numbers
 */
static
int is_odd(
	int x)
{
	return x&1;
}

/**
 * @brief returns 1 if x is even, 0 otherwise; works also for negative numbers
 */
static
int is_even(
	int x)
{
	return 1&~x;
}

/**
 * @brief returns closest even integer not larger than x; works also for negative numbers
 */
static
int to_even(
	int x)
{
	return x&~1;
}

/**
 * @brief returns closest odd integer not larger than x; works also for negative numbers
 */
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
	assert( NULL != dst && NULL != src );

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
	assert( NULL != dst && NULL != src );

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
	assert( NULL != ptr );

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
	assert( NULL != ptr );

	for(int y = 0; y < size_i_big_y; y++)
		for(int x = 0; x < size_i_big_x; x++)
			*addr2_s(ptr, y, x, stride_x, stride_y) = dwt_util_test_image_value_s(x, y, rand);
}

void dwt_util_test_image_zero_s(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_i_big_x,
	int size_i_big_y)
{
	assert( NULL != ptr );

	for(int y = 0; y < size_i_big_y; y++)
		for(int x = 0; x < size_i_big_x; x++)
			*addr2_s(ptr, y, x, stride_x, stride_y) = 0.0f;
}

void dwt_util_alloc_image(
	void **pptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y)
{
	assert( NULL != pptr );

	UNUSED(stride_y);
	UNUSED(size_o_big_x);

	*pptr = malloc(stride_x*size_o_big_y);
	if(NULL == *pptr)
		abort();
}

void dwt_util_free_image(
	void **pptr)
{
	assert( pptr != NULL );

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
	assert( ptr1 != NULL && ptr2 != NULL && size_i_big_x >= 0 && size_i_big_y >= 0 );

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
	assert( ptr1 != NULL && ptr2 != NULL && size_i_big_x >= 0 && size_i_big_y >= 0 );

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

void dwt_cdf53_f_d(
	const double *src,
	double *dst,
	double *tmp,
	int N)
{
	dwt_cdf53_f_ex_d(
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

void dwt_cdf53_f_s(
	const float *src,
	float *dst,
	float *tmp,
	int N)
{
	dwt_cdf53_f_ex_s(
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

void dwt_cdf53_i_d(
	const double *src,
	double *dst,
	double *tmp,
	int N)
{
	dwt_cdf53_i_ex_d(
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

void dwt_cdf53_i_s(
	const float *src,
	float *dst,
	float *tmp,
	int N)
{
	dwt_cdf53_i_ex_s(
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

void dwt_cdf53_f_ex_d(
	const double *src,
	double *dst_l,
	double *dst_h,
	double *tmp,
	int N)
{
	dwt_cdf53_f_ex_stride_d(
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

void dwt_cdf53_f_ex_s(
	const float *src,
	float *dst_l,
	float *dst_h,
	float *tmp,
	int N)
{
	dwt_cdf53_f_ex_stride_s(
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
	assert( N >= 0 && NULL != src && NULL != dst_l && NULL != dst_h && NULL != tmp && 0 != stride );

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

void dwt_cdf53_f_ex_stride_d(
	const double *src,
	double *dst_l,
	double *dst_h,
	double *tmp,
	int N,
	int stride)
{
	assert( N >= 0 && NULL != src && NULL != dst_l && NULL != dst_h && NULL != tmp && 0 != stride );

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst_l[0] = src[0] * dwt_cdf53_s1_d;
		return;
	}

	// copy src into tmp
	dwt_util_memcpy_stride_d(tmp, sizeof(double), src, stride, N);

	// predict 1 + update 1
	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] -= dwt_cdf53_p1_d * (tmp[i-1] + tmp[i+1]);

	if(is_odd(N))
		tmp[N-1] += 2 * dwt_cdf53_u1_d * tmp[N-2];
	else
		tmp[N-1] -= 2 * dwt_cdf53_p1_d * tmp[N-2];

	tmp[0] += 2 * dwt_cdf53_u1_d * tmp[1];

	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] += dwt_cdf53_u1_d * (tmp[i-1] + tmp[i+1]);

	// scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf53_s1_d;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf53_s2_d;

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
	assert( steps >= 0 );

	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		// TODO: fix bordering element due to non-64-bits alignment hack
		float *arr_local = calc_temp_step(arr, w);

		if( scaling < 0 )
		{
			for(int s = 0; s < steps; s++)
			{
				arr_local[4+s*2] *= 1/zeta;
			}
		
			for(int s = 0; s < steps; s++)
			{
				arr_local[5+s*2] *= zeta;
			}
		}

		const float coeffs[4] = {delta, gamma, beta, alpha};

		for(int off = 4; off >= 1; off--)
		{
			float *out = arr_local+off;

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
				arr_local[0+s*2] *= 1/zeta;
			}

			for(int s = 0; s < steps; s++)
			{
				arr_local[1+s*2] *= zeta;
			}
		}
	}
}

static
int is_aligned_8(
	const void *ptr)
{
	return ( (intptr_t)ptr&(intptr_t)(8-1) ) ? 0 : 1;
}

/**
 * @returns 0 when not aligned or 1 when aligned
 */
int dwt_util_is_aligned_8(
	const void *ptr)
{
	return is_aligned_8(ptr);
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
	FUNC_BEGIN;

	assert( steps >= 0 );

#ifdef microblaze
	UNUSED(scaling);
	UNUSED(alpha);
	UNUSED(beta);
	UNUSED(gamma);
	UNUSED(delta);
	UNUSED(zeta);

	assert( steps <= (BANK_SIZE - 4 + ( is_aligned_8(arr) ? 0 : -2 ) ) / 2 );

	// HACK: due to fixing DMA transfers of addr not aligned to 64-bits => accessing outside of my memory!
	const int size = 2*steps + 4 + ( is_aligned_8(arr) ? 0 : 2 );
	float *addr = arr + ( is_aligned_8(arr) ? 0 : -1 );
	const unsigned int pb_offset = ( is_aligned_8(arr) ? 0 : +1 );

	assert( is_aligned_8(addr) );
	assert( is_even(size) );

#ifndef USE_DMA
	// FIXME: remove
	WAL_CHECK( wal_mb2dmem(worker[0], 0, WAL_BCE_JSY_DMEM_A, WAL_BANK_POS(0,0), addr, size) );
#else /* USE_DMA */
	for(int w = 0; w < dwt_util_global_active_workers; w++)
	{
		// channel w according to worker ID; but each worker has independent DMA channels, thus this is not necessary
		const uint8_t ch = w;
		float *addr_local = addr + (dwt_util_global_temp_step * w); // FIXME: use function from top

		WAL_CHECK( wal_dma_configure(worker[w], ch, addr_local, 0, WAL_BCE_JSY_DMEM_A, WAL_BANK_POS(0,0), size) );
		WAL_CHECK( wal_dma_start(worker[w], ch, WAL_DMA_REQ_RD) );
	}

	for(int w = 0; w < dwt_util_global_active_workers; w++)
	{
		// HACK: wait for completing memory transfers on all 8 channels; but each worker has independent DMA channels
		while( wal_dma_isbusy(worker[w], /*WAL_DMA_MASK(ch)*/ 0x0f) )
			;
	}
#endif /* USE_DMA */

	const uint32_t steps_32 = (uint32_t)steps;
	const uint32_t pb_offset_32 = (uint32_t)pb_offset;

	// start BCE computations
	for(int w = 0; w < dwt_util_global_active_workers; w++)
	{
		WAL_CHECK( wal_mb2cmem(worker[w], WAL_CMEM_MB2PB, 0x01, &steps_32, 1) );
		WAL_CHECK( wal_mb2cmem(worker[w], WAL_CMEM_MB2PB, 0x02, &pb_offset_32, 1) );

		WAL_CHECK( wal_mb2pb(worker[w], 1) );
	}

	// wait for finishing every BCE computation
	for(int w = 0; w < dwt_util_global_active_workers; w++)
	{
		WAL_CHECK( wal_pb2mb(worker[w], NULL) );
	}

	assert( is_aligned_8(addr) );
	assert( is_even(size) );

#ifndef USE_DMA
	// FIXME: remove
	WAL_CHECK( wal_dmem2mb(worker[0], 0, WAL_BCE_JSY_DMEM_A, WAL_BANK_POS(0,0), addr, size) );
#else /* USE_DMA */
	for(int w = 0; w < dwt_util_global_active_workers; w++)
	{
		const uint8_t ch = w;
		float *addr_local = addr + (dwt_util_global_temp_step * w); // FIXME: use function from top

		// FIXME: this would suffice configure once!
		WAL_CHECK( wal_dma_configure(worker[w], ch, addr_local, 0, WAL_BCE_JSY_DMEM_A, WAL_BANK_POS(0,0), size) );
		WAL_CHECK( wal_dma_start(worker[w], ch, WAL_DMA_REQ_WR) );
	}

	for(int w = 0; w < dwt_util_global_active_workers; w++)
	{
		while( wal_dma_isbusy(worker[w], /*WAL_DMA_MASK(ch)*/ 0x0f) )
			;
	}
#endif /* USE_DMA */

	for(int w = 0; w < dwt_util_global_active_workers; w++)
	{
		float *addr_local = addr + (dwt_util_global_temp_step * w); // FIXME: use function from top

		// HACK: why +1? should be "size" only
		flush_cache(addr_local, size+1);
	}
#else /* microblaze */
	// fallback
	accel_lift_op4s_main_s(arr, steps, alpha, beta, gamma, delta, zeta, scaling);
#endif /* microblaze */

	FUNC_END;
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
	assert( N-off >= 4 );

#ifdef NDEBUG
	UNUSED(N);
#endif

	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		// TODO: fix bordering element due to non-64-bits alignment hack
		float *arr_local = calc_temp_step(arr, w);
		
		if(off)
		{
			// inv-scaling
			if( scaling < 0 )
			{
				// TODO
			}

			// alpha
			arr_local[1] += alpha*(arr_local[0]+arr_local[2]);
			arr_local[3] += alpha*(arr_local[2]+arr_local[4]);

			// beta
			arr_local[0] += 2*beta*(arr_local[1]);
			arr_local[2] += beta*(arr_local[1]+arr_local[3]);
		
			// gamma
			arr_local[1] += gamma*(arr_local[0]+arr_local[2]);
		
			// delta
			arr_local[0] += 2*delta*(arr_local[1]);

			// scaling
			if( scaling > 0)
			{
				arr_local[0] *= zeta;
			}
		}
		else
		{
			// inv-scaling
			if( scaling < 0 )
			{
				arr_local[0] *= 1/zeta;
				arr_local[1] *= zeta;
				arr_local[2] *= 1/zeta;
				arr_local[3] *= zeta;
			}

			// alpha
			arr_local[0] += 2*alpha*(arr_local[1]);
			arr_local[2] += alpha*(arr_local[1]+arr_local[3]);
			
			// beta
			arr_local[1] += beta*(arr_local[0]+arr_local[2]);
		
			// gamma
			arr_local[0] += 2*gamma*(arr_local[1]);
		
			// delta
			// none

			// scaling
			// none
		}
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
	assert( N-off >= 4 );

	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		// TODO: fix bordering element due to non-64-bits alignment hack
		float *arr_local = calc_temp_step(arr, w);

		if( is_even(N-off) )
		{
			// inv-scaling
			if( scaling < 0 )
			{
				// TODO
			}

			// alpha
			// none

			// beta
			arr_local[N-1] += 2*beta*(arr_local[N-2]);

			// gamma
			arr_local[N-2] += gamma*(arr_local[N-1]+arr_local[N-3]);

			// delta
			arr_local[N-1] += 2*delta*(arr_local[N-2]);
			arr_local[N-3] += delta*(arr_local[N-4]+arr_local[N-2]);

			// scaling
			if( scaling > 0 )
			{
				// FIXME: this is dependend on "off"
				arr_local[N-4] *= 1/zeta;
				arr_local[N-3] *= zeta;
				arr_local[N-2] *= 1/zeta;
				arr_local[N-1] *= zeta;
			}
		}
		else /* is_odd(N-off) */
		{
			// inv-scaling
			if( scaling < 0 )
			{
				arr_local[N-1] *= 1/zeta;
			}

			// alpha
			arr_local[N-1] += 2*alpha*(arr_local[N-2]);

			// beta
			arr_local[N-2] += beta*(arr_local[N-1]+arr_local[N-3]);

			// gamma
			arr_local[N-1] += 2*gamma*(arr_local[N-2]);
			arr_local[N-3] += gamma*(arr_local[N-2]+arr_local[N-4]);

			// delta
			arr_local[N-2] += delta*(arr_local[N-1]+arr_local[N-3]);
			arr_local[N-4] += delta*(arr_local[N-5]+arr_local[N-3]);

			// scaling
			if( scaling > 0 )
			{
				// FIXME: this is dependend on "off"
				arr_local[N-5] *= 1/zeta;
				arr_local[N-4] *= zeta;
				arr_local[N-3] *= 1/zeta;
				arr_local[N-2] *= zeta;
				arr_local[N-1] *= 1/zeta;
			}
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
	assert( N-off < 4 );

	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		// TODO: fix bordering element due to non-64-bits alignment hack
		float *arr_local = calc_temp_step(arr, w);

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
				arr_local[1] += 2*alpha*(arr_local[0]);

				// beta
				arr_local[0] += 2*beta*(arr_local[1]);

				// gamma
				arr_local[1] += 2*gamma*(arr_local[0]);

				// delta
				arr_local[0] += 2*delta*(arr_local[1]);

				// scaling
				if( scaling > 0 )
				{
					arr_local[0] *= zeta;
					arr_local[1] *= 1/zeta;
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
				arr_local[1] += alpha*(arr_local[0]+arr_local[2]);

				// beta
				arr_local[0] += 2*beta*(arr_local[1]);
				arr_local[2] += 2*beta*(arr_local[1]);

				// gamma
				arr_local[1] += gamma*(arr_local[0]+arr_local[2]);

				// delta
				arr_local[0] += 2*delta*(arr_local[1]);
				arr_local[2] += 2*delta*(arr_local[1]);

				// scaling
				if( scaling > 0 )
				{
					arr_local[0] *= zeta;
					arr_local[1] *= 1/zeta;
					arr_local[2] *= zeta;
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
				arr_local[1] += alpha*(arr_local[0]+arr_local[2]);
				arr_local[3] += 2*alpha*(arr_local[2]);

				// beta
				arr_local[0] += 2*beta*(arr_local[1]);
				arr_local[2] += beta*(arr_local[1]+arr_local[3]);

				// gamma
				arr_local[1] += gamma*(arr_local[0]+arr_local[2]);
				arr_local[3] += 2*gamma*(arr_local[2]);

				// delta
				arr_local[0] += 2*delta*(arr_local[1]);
				arr_local[2] += delta*(arr_local[1]+arr_local[3]);

				// scaling
				if( scaling > 0 )
				{
					arr_local[0] *= zeta;
					arr_local[1] *= 1/zeta;
					arr_local[2] *= zeta;
					arr_local[3] *= 1/zeta;
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
					arr_local[0] *= 1/zeta;
					arr_local[1] *= zeta;
				}

				// alpha
				arr_local[0] += 2*alpha*(arr_local[1]);

				// beta
				arr_local[1] += 2*beta*(arr_local[0]);

				// gamma
				arr_local[0] += 2*gamma*(arr_local[1]);

				// delta
				arr_local[1] += 2*delta*(arr_local[0]);

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
					arr_local[0] *= 1/zeta;
					arr_local[1] *= zeta;
					arr_local[2] *= 1/zeta;
				}

				// alpha
				arr_local[0] += 2*alpha*(arr_local[1]);
				arr_local[2] += 2*alpha*(arr_local[1]);

				// beta
				arr_local[1] += beta*(arr_local[0]+arr_local[2]);

				// gamma
				arr_local[0] += 2*gamma*(arr_local[1]);
				arr_local[2] += 2*gamma*(arr_local[1]);

				// delta
				arr_local[1] += delta*(arr_local[0]+arr_local[2]);

				// scaling
				if( scaling > 0 )
				{
					// TODO
				}
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
	assert( len >= 2 );
	assert( 0 == off || 1 == off );

	if( len-off < 4 )
	{
		accel_lift_op4s_short_s(arr, off, len, alpha, beta, gamma, delta, zeta, scaling);
	}
	else
	{
		accel_lift_op4s_prolog_s(arr, off, len, alpha, beta, gamma, delta, zeta, scaling);

		if(1 == dwt_util_global_accel_type[0])
		{
			const int max_inner_len = to_even(BANK_SIZE) - 4 + ( is_aligned_8(arr+off) ? 0 : -2);
			const int inner_len = to_even(len-off) - 4; // FIXME: maybe -2? try on odd lengths and another decompositions like small image in big one... opencv example should test it
			const int blocks = inner_len / max_inner_len;
			// full length blocks
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
				if( steps > /* FIXME: 96 */ 10 )
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
		else if(3 == dwt_util_global_accel_type[0])
		{
			off = 0;
			accel_lift_op4s_main_pb_s(arr+off, (to_even(len-off)-4)/2, alpha, beta, gamma, delta, zeta, scaling);
		}
		else
			dwt_util_abort(); // unsupported value

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
	assert( N >= 0 && NULL != src && NULL != dst_l && NULL != dst_h && NULL != tmp && 0 != stride );

	// fix for small N
	// FIXME: this should be parallel too!
	if(N < 2)
	{
		if(1 == N)
			dst_l[0] = src[0] * dwt_cdf97_s1_s;
		return;
	}

	// copy src into tmp
	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		float *tmp_local = calc_temp_step(tmp, w);
		const float *src_local = calc_data_step_const(src, w);

		dwt_util_memcpy_stride_s(tmp_local, sizeof(float), src_local, stride, N);
	}

	// TODO: fix bordering element due to non-64-bits alignment hack in tmp[]
	accel_lift_op4s_s(tmp, 1, N, -dwt_cdf97_p1_s, dwt_cdf97_u1_s, -dwt_cdf97_p2_s, dwt_cdf97_u2_s, dwt_cdf97_s1_s, +1);

	// copy tmp into dst
	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		float *tmp_local = calc_temp_step(tmp, w);
		float *dst_l_local = calc_data_step(dst_l, w);
		float *dst_h_local = calc_data_step(dst_h, w);

		dwt_util_memcpy_stride_s(dst_l_local, stride, tmp_local+0, 2*sizeof(float),  ceil_div2(N));
		dwt_util_memcpy_stride_s(dst_h_local, stride, tmp_local+1, 2*sizeof(float), floor_div2(N));
	}
}

void dwt_cdf53_f_ex_stride_s(
	const float *src,
	float *dst_l,
	float *dst_h,
	float *tmp,
	int N,
	int stride)
{
	assert( N >= 0 && NULL != src && NULL != dst_l && NULL != dst_h && NULL != tmp && 0 != stride );

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst_l[0] = src[0] * dwt_cdf53_s1_s;
		return;
	}

	// copy src into tmp
	dwt_util_memcpy_stride_s(tmp, sizeof(float), src, stride, N);

	// predict 1 + update 1
	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] -= dwt_cdf53_p1_s * (tmp[i-1] + tmp[i+1]);

	if(is_odd(N))
		tmp[N-1] += 2 * dwt_cdf53_u1_s * tmp[N-2];
	else
		tmp[N-1] -= 2 * dwt_cdf53_p1_s * tmp[N-2];

	tmp[0] += 2 * dwt_cdf53_u1_s * tmp[1];

	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] += dwt_cdf53_u1_s * (tmp[i-1] + tmp[i+1]);

	// scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf53_s1_s;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf53_s2_s;

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

void dwt_cdf53_i_ex_d(
	const double *src_l,
	const double *src_h,
	double *dst,
	double *tmp,
	int N)
{
	dwt_cdf53_i_ex_stride_d(
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

void dwt_cdf53_i_ex_s(
	const float *src_l,
	const float *src_h,
	float *dst,
	float *tmp,
	int N)
{
	dwt_cdf53_i_ex_stride_s(
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
	assert( N >= 0 && NULL != src_l && NULL != src_h && NULL != dst && NULL != tmp && 0 != stride );

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

void dwt_cdf53_i_ex_stride_d(
	const double *src_l,
	const double *src_h,
	double *dst,
	double *tmp,
	int N,
	int stride)
{
	assert( N >= 0 && NULL != src_l && NULL != src_h && NULL != dst && NULL != tmp && 0 != stride );

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst[0] = src_l[0] * dwt_cdf53_s2_d;
		return;
	}

	// copy src into tmp
	dwt_util_memcpy_stride_d(tmp+0, 2*sizeof(double), src_l, stride,  ceil_div2(N));
	dwt_util_memcpy_stride_d(tmp+1, 2*sizeof(double), src_h, stride, floor_div2(N));

	// inverse scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf53_s2_d;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf53_s1_d;

	// backward update 1 + backward predict 1
	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] -= dwt_cdf53_u1_d * (tmp[i-1] + tmp[i+1]);

	tmp[0] -= 2 * dwt_cdf53_u1_d * tmp[1];

	if(is_odd(N))
		tmp[N-1] -= 2 * dwt_cdf53_u1_d * tmp[N-2];
	else
		tmp[N-1] += 2 * dwt_cdf53_p1_d * tmp[N-2];

	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] += dwt_cdf53_p1_d * (tmp[i-1] + tmp[i+1]);

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
	assert( N >= 0 && NULL != src_l && NULL != src_h && NULL != dst && NULL != tmp && 0 != stride );

	// fix for small N
	// FIXME: this should be parallel too!
	if(N < 2)
	{
		if(1 == N)
			dst[0] = src_l[0] * dwt_cdf97_s2_s; // FIXME: 1/zeta
		return;
	}

	// copy src into tmp
	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		float *tmp_local = calc_temp_step(tmp, w);
		const float *src_l_local = calc_data_step_const(src_l, w);
		const float *src_h_local = calc_data_step_const(src_h, w);

		dwt_util_memcpy_stride_s(tmp_local+0, 2*sizeof(float), src_l_local, stride,  ceil_div2(N));
		dwt_util_memcpy_stride_s(tmp_local+1, 2*sizeof(float), src_h_local, stride, floor_div2(N));
	}


	// TODO: fix bordering element due to non-64-bits alignment hack in tmp[]
	accel_lift_op4s_s(tmp, 0, N, -dwt_cdf97_u2_s, dwt_cdf97_p2_s, -dwt_cdf97_u1_s, dwt_cdf97_p1_s, dwt_cdf97_s1_s, -1);

	// copy tmp into dst
	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		float *tmp_local = calc_temp_step(tmp, w);
		float *dst_local = calc_data_step(dst, w);

		dwt_util_memcpy_stride_s(dst_local, stride, tmp_local, sizeof(float), N);
	}
}

void dwt_cdf53_i_ex_stride_s(
	const float *src_l,
	const float *src_h,
	float *dst,
	float *tmp,
	int N,
	int stride)
{
	assert( N >= 0 && NULL != src_l && NULL != src_h && NULL != dst && NULL != tmp && 0 != stride );

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst[0] = src_l[0] * dwt_cdf53_s2_s;
		return;
	}

	// copy src into tmp
	dwt_util_memcpy_stride_s(tmp+0, 2*sizeof(float), src_l, stride,  ceil_div2(N));
	dwt_util_memcpy_stride_s(tmp+1, 2*sizeof(float), src_h, stride, floor_div2(N));

	// inverse scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf53_s2_s;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf53_s1_s;

	// backward update 2 + backward predict 2
	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] -= dwt_cdf53_u1_s * (tmp[i-1] + tmp[i+1]);

	tmp[0] -= 2 * dwt_cdf53_u1_s * tmp[1];

	if(is_odd(N))
		tmp[N-1] -= 2 * dwt_cdf53_u1_s * tmp[N-2];
	else
		tmp[N-1] += 2 * dwt_cdf53_p1_s * tmp[N-2];

	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] += dwt_cdf53_p1_s * (tmp[i-1] + tmp[i+1]);

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
	assert( N >= 0 && N_dst_L >= 0 && N_dst_H >= 0 && 0 == ((N_dst_L-N_dst_H)&~1) && NULL != dst_l && NULL != dst_h && 0 != stride ); // FIXME: 0 == ((N_dst_L-N_dst_H)&~1)

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
	assert( N >= 0 && N_dst_L >= 0 && N_dst_H >= 0 && 0 == ((N_dst_L-N_dst_H)&~1) && NULL != dst_l && NULL != dst_h && 0 != stride ); // FIXME: 0 == ((N_dst_L-N_dst_H)&~1)

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
	assert( N >= 0 && N_dst >= 0 && NULL != dst_l && 0 != stride );

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
	assert( N >= 0 && N_dst >= 0 && NULL != dst_l && 0 != stride );

	const float zero = 0;

	dwt_util_memcpy_stride_s(
		addr1_s(dst_l, N, stride),
		stride,
		&zero,
		0,
		N_dst - N);
}

void dwt_util_switch_op(
	enum dwt_op op)
{
	FUNC_BEGIN;

#ifdef microblaze
	//WAL_CHECK( wal_mb2pb(worker, 0) );

	//WAL_CHECK( wal_bce_jk_sync_operation(worker) );

	for(int w = 0; w < dwt_util_global_total_workers; w++)
	{
		WAL_CHECK( wal_reset_worker(worker[w]) );
	}

	switch(op)
	{
		case DWT_OP_LIFT4SA:
		{
			for(int w = 0; w < dwt_util_global_total_workers; w++)
			{
				WAL_CHECK( wal_start_operation(worker[w], WAL_PBID_P0) );
			}

			float alpha = -dwt_cdf97_p1_s,
				beta = dwt_cdf97_u1_s,
				gamma = -dwt_cdf97_p2_s,
				delta = dwt_cdf97_u2_s,
				zeta = dwt_cdf97_s1_s;

			const int size = 12;
			// FIXME: for these coeeficients, use memory bank "D"
			const float coeffs[12] = {delta, 0.0f, gamma, 0.0f, beta, 0.0f, alpha, 0.0f, zeta, 0.0f, 1/zeta, 0.0f};
			float *addr = dwt_util_allocate_vec_s(size);
			if(!addr)
			{
				dwt_util_log(LOG_ERR, "Failed to allocate vector of %i floats.\n", size);
				dwt_util_abort();
			}
			dwt_util_copy_vec_s(coeffs, addr, size);

			assert( is_even(size) );
			assert( is_aligned_8(addr) );

#ifndef USE_DMA
			// FIXME: remove
			WAL_CHECK( wal_mb2dmem(worker[0], 0, WAL_BCE_JSY_DMEM_B, WAL_BANK_POS(0,0), addr, size) );
#else
			for(int w = 0; w < dwt_util_global_total_workers; w++)
			{
				WAL_CHECK( wal_dma_configure(worker[w], 0, addr, 0, WAL_BCE_JSY_DMEM_B, 0, size) );
				WAL_CHECK( wal_dma_start(worker[w], 0, WAL_DMA_REQ_RD) );
				while( wal_dma_isbusy(worker[w], 0x01) )
					;
			}
#endif
			free(addr);
		}
		break;
		case DWT_OP_LIFT4SB:
		{
			for(int w = 0; w < dwt_util_global_total_workers; w++)
			{
				WAL_CHECK( wal_start_operation(worker[w], WAL_PBID_P1) );
			}

			float alpha = -dwt_cdf97_u2_s,
				beta = dwt_cdf97_p2_s,
				gamma = -dwt_cdf97_u1_s,
				delta = dwt_cdf97_p1_s,
				zeta = dwt_cdf97_s1_s;

			const int size = 12;
			// FIXME: for these coeeficients, use memory bank "D"
			const float coeffs[12] = {delta, 0.0f, gamma, 0.0f, beta, 0.0f, alpha, 0.0f, zeta, 0.0f, 1/zeta, 0.0f};
			float *addr = dwt_util_allocate_vec_s(size);
			if(!addr)
			{
				dwt_util_log(LOG_ERR, "Failed to allocate vector of %i floats.\n", size);
				dwt_util_abort();
			}
			dwt_util_copy_vec_s(coeffs, addr, size);

			assert( is_even(size) );
			assert( is_aligned_8(addr) );

#ifndef USE_DMA
			// FIXME: remove
			WAL_CHECK( wal_mb2dmem(worker[0], 0, WAL_BCE_JSY_DMEM_B, WAL_BANK_POS(0,0), addr, size) );
#else
			for(int w = 0; w < dwt_util_global_total_workers; w++)
			{
				WAL_CHECK( wal_dma_configure(worker[w], 0, addr, 0, WAL_BCE_JSY_DMEM_B, 0, size) );
				WAL_CHECK( wal_dma_start(worker[w], 0, WAL_DMA_REQ_RD) );
				while( wal_dma_isbusy(worker[w], 0x01) )
					;
			}
#endif
			free(addr);
		}
		break;
		default:
			dwt_util_abort();
	}
#else
	UNUSED(op);
#endif

	FUNC_END;
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

void dwt_cdf53_2f_d(
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
			dwt_cdf53_f_ex_stride_d(
				addr2_d(ptr,y,0,stride_x,stride_y),
				addr2_d(ptr,y,0,stride_x,stride_y),
				addr2_d(ptr,y,size_o_dst_x,stride_x,stride_y),
				temp,
				size_i_src_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_src_x; x++)
			dwt_cdf53_f_ex_stride_d(
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
	FUNC_BEGIN;

	dwt_util_switch_op(DWT_OP_LIFT4SA); // FIXME

	const int size_o_big_min = min(size_o_big_x,size_o_big_y);
	const int size_o_big_max = max(size_o_big_x,size_o_big_y);

	// FIXME: elliminate
	const int active_workers = dwt_util_get_num_workers();

#ifdef microblaze
	// FIXME: wrap this assignment
	dwt_util_global_temp_step = size_o_big_max;
#endif

	float temp[size_o_big_max*active_workers];
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

#ifdef microblaze
		// FIXME: temporarily, only power of two sizes
		assert( size_o_src_y == pow2_ceil_log2(size_o_src_y) );
		assert( size_o_src_x == pow2_ceil_log2(size_o_src_x) );
#endif

		const int threads_segment_y = ceil_div(size_o_src_y, dwt_util_get_num_threads());
		const int threads_segment_x = ceil_div(size_o_src_x, dwt_util_get_num_threads());
		const int workers_segment_y = ceil_div(size_o_src_y, dwt_util_get_num_workers());
		const int workers_segment_x = ceil_div(size_o_src_x, dwt_util_get_num_workers());

#ifdef microblaze
		// FIXME: wrap this by function
		dwt_util_global_data_step = (intptr_t)addr2_s(ptr,workers_segment_y,0,stride_x,stride_y) - (intptr_t)addr2_s(ptr,0,0,stride_x,stride_y);
#endif
		//#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_src_y, omp_get_num_threads()))
		#pragma omp parallel for private(temp) schedule(static, threads_segment_y)
		for(int y = 0; y < workers_segment_y; y++)
			dwt_cdf97_f_ex_stride_s(
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,size_o_dst_x,stride_x,stride_y),
				temp,
				size_i_src_x,
				stride_y);
#ifdef microblaze
		dwt_util_global_data_step = (intptr_t)addr2_s(ptr,0,workers_segment_x,stride_x,stride_y) - (intptr_t)addr2_s(ptr,0,0,stride_x,stride_y);
#endif
		//#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
		#pragma omp parallel for private(temp) schedule(static, threads_segment_x)
		for(int x = 0; x < /*size_o_src_x/active_workers*/workers_segment_x; x++)
			dwt_cdf97_f_ex_stride_s(
				addr2_s(ptr,0,x,stride_x,stride_y),
				addr2_s(ptr,0,x,stride_x,stride_y),
				addr2_s(ptr,size_o_dst_y,x,stride_x,stride_y),
				temp,
				size_i_src_y,
				stride_x);

		if(zero_padding)
		{
			//#pragma omp parallel for schedule(static, ceil_div(size_o_src_y, omp_get_num_threads()))
			#pragma omp parallel for schedule(static, threads_segment_y)
			for(int y = 0; y < size_o_src_y; y++)
				dwt_zero_padding_f_stride_s(
					addr2_s(ptr,y,0,stride_x,stride_y),
					addr2_s(ptr,y,size_o_dst_x,stride_x,stride_y),
					size_i_src_x,
					size_o_dst_x,
					size_o_src_x-size_o_dst_x,
					stride_y);
			//#pragma omp parallel for schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
			#pragma omp parallel for schedule(static, threads_segment_x)
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

	FUNC_END;
}

void dwt_cdf53_2f_s(
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
			dwt_cdf53_f_ex_stride_s(
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,size_o_dst_x,stride_x,stride_y),
				temp,
				size_i_src_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_src_x; x++)
			dwt_cdf53_f_ex_stride_s(
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

void dwt_cdf53_2i_d(
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
			dwt_cdf53_i_ex_stride_d(
				addr2_d(ptr,y,0,stride_x,stride_y),
				addr2_d(ptr,y,size_o_src_x,stride_x,stride_y),
				addr2_d(ptr,y,0,stride_x,stride_y),
				temp,
				size_i_dst_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_dst_x; x++)
			dwt_cdf53_i_ex_stride_d(
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
	FUNC_BEGIN;

	dwt_util_switch_op(DWT_OP_LIFT4SB); // FIXME

	const int size_o_big_min = min(size_o_big_x,size_o_big_y);
	const int size_o_big_max = max(size_o_big_x,size_o_big_y);

	const int active_workers = dwt_util_get_num_workers();

#ifdef microblaze
	dwt_util_global_temp_step = size_o_big_max;
#endif

	float temp[size_o_big_max*active_workers];
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

#ifdef microblaze
		// FIXME
		assert( size_o_dst_y == pow2_ceil_log2(size_o_dst_y) );
		assert( size_o_dst_x == pow2_ceil_log2(size_o_dst_x) );
#endif

		const int threads_segment_y = ceil_div(size_o_dst_y, dwt_util_get_num_threads());
		const int threads_segment_x = ceil_div(size_o_dst_x, dwt_util_get_num_threads());
		const int workers_segment_y = ceil_div(size_o_dst_y, dwt_util_get_num_workers());
		const int workers_segment_x = ceil_div(size_o_dst_x, dwt_util_get_num_workers());

#ifdef microblaze
		dwt_util_global_data_step = (intptr_t)addr2_s(ptr,workers_segment_y,0,stride_x,stride_y) - (intptr_t)addr2_s(ptr,0,0,stride_x,stride_y);
#endif

		//#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_dst_y, omp_get_num_threads()))
		#pragma omp parallel for private(temp) schedule(static, threads_segment_y)
		for(int y = 0; y < workers_segment_y; y++)
			dwt_cdf97_i_ex_stride_s(
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,size_o_src_x,stride_x,stride_y),
				addr2_s(ptr,y,0,stride_x,stride_y),
				temp,
				size_i_dst_x,
				stride_y);
#ifdef microblaze
		dwt_util_global_data_step = (intptr_t)addr2_s(ptr,0,workers_segment_x,stride_x,stride_y) - (intptr_t)addr2_s(ptr,0,0,stride_x,stride_y);
#endif
		//#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
		#pragma omp parallel for private(temp) schedule(static, threads_segment_x)
		for(int x = 0; x < workers_segment_x; x++)
			dwt_cdf97_i_ex_stride_s(
				addr2_s(ptr,0,x,stride_x,stride_y),
				addr2_s(ptr,size_o_src_y,x,stride_x,stride_y),
				addr2_s(ptr,0,x,stride_x,stride_y),
				temp,
				size_i_dst_y,
				stride_x);

		if(zero_padding)
		{
			//#pragma omp parallel for schedule(static, ceil_div(size_o_dst_y, omp_get_num_threads()))
			#pragma omp parallel for schedule(static, threads_segment_y)
			for(int y = 0; y < size_o_dst_y; y++)
				dwt_zero_padding_i_stride_s(
					addr2_s(ptr,y,0,stride_x,stride_y),
					size_i_dst_x,
					size_o_dst_x,
					stride_y);
			//#pragma omp parallel for schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
			#pragma omp parallel for schedule(static, threads_segment_x)
			for(int x = 0; x < size_o_dst_x; x++)
				dwt_zero_padding_i_stride_s(
					addr2_s(ptr,0,x,stride_x,stride_y),
					size_i_dst_y,
					size_o_dst_y,
					stride_x);
		}

		j--;
	}

	FUNC_END;
}

void dwt_cdf53_2i_s(
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
			dwt_cdf53_i_ex_stride_s(
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,size_o_src_x,stride_x,stride_y),
				addr2_s(ptr,y,0,stride_x,stride_y),
				temp,
				size_i_dst_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_dst_x; x++)
			dwt_cdf53_i_ex_stride_s(
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
#ifdef ENABLE_TIME_GETTIMEOFDAY
	return DWT_TIME_GETTIMEOFDAY;
#endif
#ifdef ENABLE_TIME_IOCTL_RTC
	return DWT_TIME_IOCTL_RTC;
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
		case DWT_TIME_CLOCK_GETTIME_REALTIME:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_REALTIME
			return_freq = (dwt_clock_t)1000000000;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_MONOTONIC:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_MONOTONIC
			return_freq = (dwt_clock_t)1000000000;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_MONOTONIC_RAW:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_MONOTONIC_RAW
			return_freq = (dwt_clock_t)1000000000;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID
			return_freq = (dwt_clock_t)1000000000;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID
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
			return_freq = (dwt_clock_t)sysconf(_SC_CLK_TCK);
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
		case DWT_TIME_IOCTL_RTC:
		{
#ifdef ENABLE_TIME_IOCTL_RTC
			return_freq = (dwt_clock_t)1;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETTIMEOFDAY:
		{
#ifdef ENABLE_TIME_GETTIMEOFDAY
			return_freq = (dwt_clock_t)1000000000;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE_SELF:
		{
#ifdef ENABLE_TIME_GETRUSAGE_SELF
			return_freq = (dwt_clock_t)1000000000;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE_CHILDREN:
		{
#ifdef ENABLE_TIME_GETRUSAGE_CHILDREN
			return_freq = (dwt_clock_t)1000000000;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE_THREAD:
		{
#ifdef ENABLE_TIME_GETRUSAGE_THREAD
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
			clockid_t clk_id = CLOCK_REALTIME;

			struct timespec ts;

			if(clock_gettime(clk_id, &ts))
				abort();

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_REALTIME:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_REALTIME
			clockid_t clk_id = CLOCK_REALTIME;

			struct timespec ts;

			if(clock_gettime(clk_id, &ts))
				abort();

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_MONOTONIC:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_MONOTONIC
			clockid_t clk_id = CLOCK_MONOTONIC;

			struct timespec ts;

			if(clock_gettime(clk_id, &ts))
				abort();

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_MONOTONIC_RAW:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_MONOTONIC_RAW
			clockid_t clk_id = CLOCK_MONOTONIC_RAW;

			struct timespec ts;

			if(clock_gettime(clk_id, &ts))
				abort();

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID
			clockid_t clk_id = CLOCK_PROCESS_CPUTIME_ID;

			struct timespec ts;

			if(clock_gettime(clk_id, &ts))
				abort();

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID
			clockid_t clk_id = CLOCK_THREAD_CPUTIME_ID;

			struct timespec ts;

			if(clock_gettime(clk_id, &ts))
				abort();

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
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
			struct tms tms_i;

			if( (clock_t)-1 == times(&tms_i) )
				abort();

			return_time = (dwt_clock_t)tms_i.tms_utime;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE:
		{
#ifdef ENABLE_TIME_GETRUSAGE
			int who = RUSAGE_SELF;

			struct rusage rusage_i;
			struct timespec ts;

			if( -1 == getrusage(who, &rusage_i) )
				abort();

			TIMEVAL_TO_TIMESPEC(&rusage_i.ru_utime, &ts);

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_IOCTL_RTC:
		{
#ifdef ENABLE_TIME_IOCTL_RTC
			int fd = open("/dev/rtc", O_RDONLY|O_NONBLOCK);
			if( -1 == fd )
				abort();

			struct rtc_time rtc_time_i;

			if( -1 == ioctl(fd, RTC_RD_TIME, &rtc_time_i) )
				abort();

			if( -1 == close(fd) )
				abort();

			time_t time = mktime( (struct tm *)&rtc_time_i );
			if( (time_t)-1 == time )
				abort();

			return_time = (dwt_clock_t)time;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETTIMEOFDAY:
		{
#ifdef ENABLE_TIME_GETTIMEOFDAY
			struct timeval tv;
			struct timespec ts;

			if( -1 == gettimeofday(&tv, NULL) )
				abort();

			TIMEVAL_TO_TIMESPEC(&tv, &ts);

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE_SELF:
		{
#ifdef ENABLE_TIME_GETRUSAGE_SELF
			int who = RUSAGE_SELF;

			struct rusage rusage_i;
			struct timespec ts;

			if( -1 == getrusage(who, &rusage_i) )
				abort();

			TIMEVAL_TO_TIMESPEC(&rusage_i.ru_utime, &ts);

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE_CHILDREN:
		{
#ifdef ENABLE_TIME_GETRUSAGE_CHILDREN
			int who = RUSAGE_CHILDREN;

			struct rusage rusage_i;
			struct timespec ts;

			if( -1 == getrusage(who, &rusage_i) )
				abort();

			TIMEVAL_TO_TIMESPEC(&rusage_i.ru_utime, &ts);

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
#else
			abort();
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE_THREAD:
		{
#ifdef ENABLE_TIME_GETRUSAGE_THREAD
			int who = RUSAGE_THREAD;

			struct rusage rusage_i;
			struct timespec ts;

			if( -1 == getrusage(who, &rusage_i) )
				abort();

			TIMEVAL_TO_TIMESPEC(&rusage_i.ru_utime, &ts);

			return_time = (dwt_clock_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
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

int dwt_util_clock_available(
	int type)
{
	switch(type)
	{
		case DWT_TIME_CLOCK_GETTIME:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME
			return 0;
#endif
		}
		break;

		case DWT_TIME_CLOCK_GETTIME_REALTIME:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_REALTIME
			return 0;
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_MONOTONIC:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_MONOTONIC
			return 0;
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_MONOTONIC_RAW:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_MONOTONIC_RAW
			return 0;
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID
			return 0;
#endif
		}
		break;
		case DWT_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID:
		{
#ifdef ENABLE_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID
			return 0;
#endif
		}
		break;
		case DWT_TIME_CLOCK:
		{
#ifdef ENABLE_TIME_CLOCK
			return 0;
#endif
		}
		break;
		case DWT_TIME_TIMES:
		{
#ifdef ENABLE_TIME_TIMES
			return 0;
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE:
		{
#ifdef ENABLE_TIME_GETRUSAGE
			return 0;
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE_SELF:
		{
#ifdef ENABLE_TIME_GETRUSAGE_SELF
			return 0;
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE_CHILDREN:
		{
#ifdef ENABLE_TIME_GETRUSAGE_CHILDREN
			return 0;
#endif
		}
		break;
		case DWT_TIME_GETRUSAGE_THREAD:
		{
#ifdef ENABLE_TIME_GETRUSAGE_THREAD
			return 0;
#endif
		}
		break;
		case DWT_TIME_GETTIMEOFDAY:
		{
#ifdef ENABLE_TIME_GETTIMEOFDAY
			return 0;
#endif
		}
		break;
		case DWT_TIME_IOCTL_RTC:
		{
#ifdef ENABLE_TIME_IOCTL_RTC
			return 0;
#endif
		}
		break;
		case DWT_TIME_AUTOSELECT:
		{
#ifdef ENABLE_TIME_AUTOSELECT
			return 0;
#endif
		}
		break;
	}

	return -1;
}

void dwt_util_wait(int ms)
{
	assert( ms > 0 );

	const int type = dwt_util_clock_autoselect();

	const dwt_clock_t freq = dwt_util_get_frequency(type);

	const dwt_clock_t start = dwt_util_get_clock(type);

	while( 1000.0f * (dwt_util_get_clock(type) - start) / freq < (float)ms )
		;
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
#else /* _OPENMP */
	return 1;
#endif /* _OPENMP */
}

int dwt_util_get_max_workers()
{
#ifdef microblaze
	return dwt_util_global_total_workers;
#else /* microblaze */
	return 1;
#endif /* microblaze */
}

void dwt_util_set_num_threads(
	int num_threads)
{
	assert( num_threads > 0 );

#ifdef _OPENMP
	omp_set_num_threads(num_threads);
#else
	UNUSED(num_threads);
#endif
}

void dwt_util_set_num_workers(
	int num_workers)
{
	assert( num_workers > 0 );

#ifdef microblaze
	dwt_util_global_active_workers = num_workers;
#else
	UNUSED(num_workers);
#endif
}

int dwt_util_get_num_threads()
{
#ifdef _OPENMP
	int num_threads;
	#pragma omp parallel
	{
		#pragma omp master
		{
			num_threads = omp_get_num_threads();
		}
	}
	return num_threads;
#else
	return 1;
#endif
}

int dwt_util_get_num_workers()
{
#ifdef microblaze
	return dwt_util_global_active_workers;
#else
	return 1;
#endif
}

void dwt_util_init()
{
	FUNC_BEGIN;

#ifdef microblaze
	for(int w = 0; w < dwt_util_global_total_workers; w++)
	{
		WAL_CHECK( wal_init_worker(worker[w]) );

		WAL_CHECK( wal_set_firmware(worker[w], /*WAL_PBID_P0*/DWT_OP_LIFT4SA, fw_fp01_lift4sa, -1) );

		WAL_CHECK( wal_set_firmware(worker[w], /*WAL_PBID_P1*/DWT_OP_LIFT4SB, fw_fp01_lift4sb, -1) );

		// TODO: call switch_op()

		WAL_CHECK( wal_reset_worker(worker[w]) );

		WAL_CHECK( wal_start_operation(worker[w], WAL_PBID_P0) );
	}

	dwt_util_set_accel(1);
#endif /* microblaze */

	FUNC_END;
}

void dwt_util_finish()
{
	FUNC_BEGIN;

#ifdef microblaze
	for(int w = 0; w < dwt_util_global_total_workers; w++)
	{
		//WAL_CHECK( wal_mb2pb(worker, 0) );

		//WAL_CHECK( wal_bce_jk_sync_operation(worker) );

		WAL_CHECK( wal_done_worker(worker[w]) );
	}
#endif

	FUNC_END;
}

void dwt_util_abort()
{
	FUNC_BEGIN;

#ifdef microblaze
	for(int w = 0; w < dwt_util_global_total_workers; w++)
	{
		// FIXME: is this legal? although the operation was not running?
		wal_end_operation(worker[w]);

		// deinitialize worker
		wal_done_worker(worker[w]);
	}

	// abort program
	abort();
#endif /* microblaze */

	FUNC_END;
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
	assert( max_value != 0.0f && size_i_big_x >= 0 && size_i_big_y >= 0 );

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

#define iszero(x) (fpclassify(x) == FP_ZERO)

int dwt_util_is_normal_or_zero_i(const float *a)
{
	if( isnormal(*a) || iszero(*a) )
		return 1;

	return 0;
}

int dwt_util_is_normal_or_zero(float a)
{
	return dwt_util_is_normal_or_zero_i(&a);
}

// FIXME: this should not abort the program, only error code should be returned
void dwt_util_cmp_s_i(const float *a, const float *b)
{
	assert( a );
	assert( b );
	
	const float eps = 1e-4; // FIXME: magic constant

	if( !dwt_util_is_normal_or_zero_i(a) || !dwt_util_is_normal_or_zero_i(b) )
	{
		printf("ERROR: %f or %f is not normal nor zero!\n", *a, *b);
		dwt_util_abort();
	}

	if( fabsf( (*a) - (*b) ) > eps )
	{
		printf("ERROR: %f should be %f!\n", *a, *b);
		dwt_util_abort();
	}
}

// FIXME: this should not abort the program, only error code should be returned
void dwt_util_cmp_s(float a, float b)
{
	dwt_util_cmp_s_i(&a, &b);
}

void dwt_util_generate_vec_s(float *addr, int size)
{
	for(int i=0; i<size; i++)
		addr[i] = (float)i;

	for(int i=0; i<size; i++)
	{
		dwt_util_cmp_s(addr[i], (float)i);
	}
}

float *dwt_util_allocate_vec_s(int size)
{
	assert( is_even(size) );

	float *addr = (float *)0;

	// FIXME: possibly infinite loop!
	do
	{
		// free(addr); // FIXME: freezes in an infinite loop
		addr = (float *)malloc( sizeof(float) * size );
	}
	while( !is_aligned_8(addr) );

	return addr;
}

void dwt_util_zero_vec_s(float *addr, int size)
{
	for(int i=0; i<size; i++)
		addr[i] = (float)0;

	for(int i=0; i<size; i++)
	{
		dwt_util_cmp_s(addr[i], (float)0);
	}
}

void dwt_util_copy_vec_s(const float *src, float *dst, int size)
{
	dwt_util_memcpy_stride_s(dst, sizeof(dst[0]), src, sizeof(src[0]), size);

	for(int i=0; i<size; i++)
	{
		dwt_util_cmp_s(dst[i], src[i]);
	}
}

void dwt_util_cmp_vec_s(const float *a, const float *b, int size)
{
	for(int i = size-1; i >= 0; i--)
	{
		dwt_util_cmp_s(a[i], b[i]);
	}
}

void dwt_util_print_vec_s(const float *addr, int size)
{
	printf("[ ");
	for(int i=0; i<size; i++)
		printf("%f ", addr[i]);
	printf("]\n");
}

void dwt_util_test()
{
	for(int i = 2; i <= BANK_SIZE; i *= 2)
	{
		dwt_util_log(LOG_TEST, "alloc vector of %i floats...\n", i);
		float *addr = dwt_util_allocate_vec_s(i);
		if( !addr )
		{
			dwt_util_log(LOG_ERR, "Failed to allocate vector of %i floats.\n", i);
			dwt_util_abort();
		}
		free(addr);
		dwt_util_log(LOG_TEST, "ok\n");
	}

#ifdef microblaze
	for(int w = 0; w < dwt_util_global_total_workers; w++)
	{
		dwt_util_log(LOG_TEST, "worker %i: init worker...\n", w);
		if( wal_init_worker(worker[w]) )
			abort();
		if( wal_reset_worker(worker[w]) )
			abort();

		const int size = BANK_SIZE;

		dwt_util_log(LOG_TEST, "allocating vector of %i floats...\n", size);
		float *addr = dwt_util_allocate_vec_s(size);
		if( !addr )
			abort();

		dwt_util_generate_vec_s(addr, size);

		dwt_util_log(LOG_TEST, "making copy of vector...\n");

		float *copy = dwt_util_allocate_vec_s(size);
		if( !copy )
			abort();

		dwt_util_copy_vec_s(addr, copy, size);
		dwt_util_cmp_vec_s(addr, copy, size);
		
		dwt_util_log(LOG_TEST, "worker %i: memory transfer to BCE memory using new-style function...\n", w);

		if( wal_dma_configure(worker[w], 0, addr, 0, WAL_BCE_JSY_DMEM_A, 0, size) )
			abort();

		if( wal_dma_start(worker[w], 0, WAL_DMA_REQ_RD) )
			abort();
		while( wal_dma_isbusy(worker[w], 0x1) )
			;

		dwt_util_log(LOG_TEST, "zeroing memory...\n");

		dwt_util_zero_vec_s(addr, size);

		dwt_util_log(LOG_TEST, "worker %i: memory transfer from BCE memory using new-style function...\n", w);

		if( wal_dma_start(worker[w], 0, WAL_DMA_REQ_WR) )
			abort();
		while( wal_dma_isbusy(worker[w], 0x1) )
			;

		dwt_util_log(LOG_TEST, "flushing cache...\n");

		flush_cache(addr, size);

		dwt_util_log(LOG_TEST, "comparing with original sequence...\n");

		dwt_util_cmp_vec_s(addr, copy, size);

		dwt_util_log(LOG_TEST, "worker %i: calling done worker...\n", w);
		
		wal_done_worker(worker[w]);

		dwt_util_log(LOG_TEST, "all tests done\n");
	}
#endif
}

int dwt_util_vfprintf(FILE *stream, const char *format, va_list ap)
{
	return vfprintf(stream, format, ap);
}

int dwt_util_vprintf(const char *format, va_list ap)
{
	return dwt_util_vfprintf(stdout, format, ap);
}

int dwt_util_fprintf(FILE *stream, const char *format, ...)
{
	va_list ap;

	va_start(ap, format);
	int ret = dwt_util_vfprintf(stream, format, ap);
	va_end(ap);

	return ret;
}

int dwt_util_printf(const char *format, ...)
{
	va_list ap;

	va_start(ap, format);
	int ret = dwt_util_vprintf(format, ap);
	va_end(ap);

	return ret;
}

int dwt_util_log(
	enum dwt_util_loglevel level,
	const char *format,
	...)
{
	int ret = 0;
	FILE *stream = stderr;

	const char *prefix[] = {
		[LOG_NONE] = "",
		[LOG_DBG]  = "DEBUG: ",
		[LOG_INFO] = "INFO: ",
		[LOG_WARN] = "WARNING: ",
		[LOG_ERR]  = "ERROR: ",
		[LOG_TEST] = "TEST: ",
	};

	ret += dwt_util_fprintf(stream, prefix[level]);

	va_list ap;

	va_start(ap, format);
	ret += dwt_util_vfprintf(stream, format, ap);
	va_end(ap);

	fflush(stream);

	return ret;
}
