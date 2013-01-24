/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Fast wavelet transform implemented via lifting scheme.
 */
#include "libdwt.h"

//#define DEBUG_VERBOSE

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
		#define FUNC_BEGIN dwt_util_log(LOG_DBG, "%s ENTRY\n", __FUNCTION__)
		#define FUNC_END   dwt_util_log(LOG_DBG, "%s EXIT\n",  __FUNCTION__)
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

#ifndef BCE_DMA_CFGTABLE_NUM_ITEMS
	#warning BCE_DMA_CFGTABLE_NUM_ITEMS was not defined, using default value of 2
	#define BCE_DMA_CFGTABLE_NUM_ITEMS 2
#endif

	WAL_REGISTER_WORKER(worker0, BCE_DMA_GENERIC_4D, bce_dma_cfgtable, 0, 1, 0);
	WAL_REGISTER_WORKER(worker1, BCE_DMA_GENERIC_4D, bce_dma_cfgtable, 1, 1, 0);
	WAL_REGISTER_WORKER(worker2, BCE_DMA_GENERIC_4D, bce_dma_cfgtable, 2, 1, 0);
	WAL_REGISTER_WORKER(worker3, BCE_DMA_GENERIC_4D, bce_dma_cfgtable, 3, 1, 0);

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

	#include "firmware/fw_fp01_lift4sa.h"
	#include "firmware/fw_fp01_lift4sb.h"

	#define BANK_SIZE 1024

	#define WAL_BANK_POS(off) ( off )

	#define WAL_DMA_MASK(ch) ( 1<<(ch) )

	#ifdef NDEBUG
		/* Release build */
		#define WAL_CHECK(expr) (expr)
	#else
		/* Debug build */
		#define WAL_CHECK(expr) ( wal_abort(STRING(expr), expr) )
	#endif
#endif

/** UNUSED macro */
#define UNUSED(expr) do { (void)(expr); } while (0)

#ifdef microblaze
/** total number of workers available */
const int dwt_util_global_total_workers = BCE_DMA_CFGTABLE_NUM_ITEMS;

static int get_total_workers()
{
	return dwt_util_global_total_workers;
}

/** how many workers use for computation (can be less than total number of workers) */
int dwt_util_global_active_workers = BCE_DMA_CFGTABLE_NUM_ITEMS;

static
int get_active_workers()
{
	return dwt_util_global_active_workers;
}

static
void set_active_workers(int active_workers)
{
	dwt_util_global_active_workers = active_workers;
}

#include <stddef.h> // ptrdiff_t

/** in bytes; offset in src[] and dst[] is given by worker_id * dwt_util_global_data_step */
ptrdiff_t dwt_util_global_data_step = 0;

static
ptrdiff_t get_data_step_s()
{
	return dwt_util_global_data_step;
}

// FIXME: float *
static void set_data_step_s(ptrdiff_t data_step)
{
	dwt_util_global_data_step = data_step;
}

/** in elements; offset in temp[] is gigen by worker_id * dwt_util_global_temp_step */
int dwt_util_global_temp_step = 0;

static
int get_temp_step()
{
	return dwt_util_global_temp_step;
}

static
void set_temp_step(int temp_step)
{
	dwt_util_global_temp_step = temp_step;
}

/** memory limit due to last worker when using more workers in parallel */
intptr_t dwt_util_global_data_limit = 0;

static
void set_data_limit_s(const float *data_limit)
{
	dwt_util_global_data_limit = (intptr_t)data_limit;
}

/** active firmware in all ASVP acceleration units */
enum dwt_op dwt_util_global_active_op = DWT_OP_NONE;
#endif

static
int is_valid_data_step_s(const float *data_step)
{
#ifdef microblaze
	return (intptr_t)data_step < dwt_util_global_data_limit;
#else
	UNUSED(data_step);
	return 1;
#endif
}

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
#include <malloc.h> // memalign
#include <float.h> // FLT_EPSILON

/** OpenMP header when used */
#ifdef _OPENMP
	#include <omp.h>
#endif

/** this PACKAGE_STRING macro must be defined via compiler's command line */
#ifndef PACKAGE_STRING
	#error PACKAGE_STRING is not defined
#endif

/** quoting macros */
#define QUOTE(x) STRING(x)

/** Calc offset in src[] or dst[] array for current worker. */
static
float *calc_data_offset_s(
	float *addr,	///< pointer to array assigned to worker 0
	int worker_id	///< identifier of current worker
       )
{
#ifdef microblaze
	return (float *)( (intptr_t)addr + (get_data_step_s() * worker_id) );
#else
	UNUSED(worker_id);
	return addr;
#endif
}

/** Calc offset in src[] or dst[] array for current worker. */
static
const float *calc_data_offset_const_s(
	const float *addr,	///< pointer to array assigned to worker 0
	int worker_id		///< identifier of current worker
       )
{
#ifdef microblaze
	return (const float *)( (intptr_t)addr + (get_data_step_s() * worker_id) );
#else
	UNUSED(worker_id);
	return addr;
#endif
}

#ifdef microblaze
static inline
void flush_cache(
	intptr_t addr,	///< base address
	size_t size	///< length of memory in bytes
	)
{
	// FIXME: 4 or 8, should be detected
	const size_t dcache_line_len = 4;

	intptr_t tmp = size + (dcache_line_len * 4);
	do {
		__asm volatile (
			"wdc %0, %1;"
			:
			: "r" (addr+tmp), "r" (0)
			: "memory"
		);
		tmp -= dcache_line_len * 4;
	} while( tmp >= 0 );
}
#endif

#ifdef microblaze
static inline
void flush_cache_s(
	float *addr,
	size_t size
	)
{
	flush_cache( (intptr_t)addr, size * sizeof(float) );
}
#endif

#ifdef microblaze
void wal_abort(const char *str, int res)
{
#ifdef DEBUG_VERBOSE
	dwt_util_log(LOG_DBG, "%s = ", str);
#else
	UNUSED(str);
#endif
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

const char *dwt_util_arch()
{
#ifdef microblaze
	// HACK: ugly buggy gcc workaround
	return "microblaze";
#endif

	return QUOTE(ARCH);
}

int dwt_util_global_accel_type = 0;

static
void set_accel_type(int accel_type)
{
	dwt_util_global_accel_type = accel_type;
}

static
int get_accel_type()
{
	return dwt_util_global_accel_type;
}

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

int dwt_util_to_even(
	int x)
{
	return to_even(x);
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

int dwt_util_to_odd(
	int x)
{
	return to_odd(x);
}

static
int is_aligned_4(
	const void *ptr)
{
	return ( (intptr_t)ptr&(intptr_t)(4-1) ) ? 0 : 1;
}

static
int is_aligned_8(
	const void *ptr)
{
	return ( (intptr_t)ptr&(intptr_t)(8-1) ) ? 0 : 1;
}

static
int is_aligned_16(
	const void *ptr)
{
	return ( (intptr_t)ptr&(intptr_t)(16-1) ) ? 0 : 1;
}

static
intptr_t align_4(
	intptr_t p)
{
	return (p+(4-1))&(~(4-1));
}

static
intptr_t align_8(
	intptr_t p)
{
	return (p+(8-1))&(~(8-1));
}

static
intptr_t align_16(
	intptr_t p)
{
	return (p+(16-1))&(~(16-1));
}

intptr_t dwt_util_align_4(
	intptr_t p)
{
	return align_4(p);
}

intptr_t dwt_util_align_8(
	intptr_t p)
{
	return align_8(p);
}

intptr_t dwt_util_align_16(
	intptr_t p)
{
	return align_16(p);
}

/** Calc offset in temp[] array for current worker. Preserves alignment on 8 bytes. */
static
float *calc_temp_offset_s(
	float *addr,	///< pointer to array assigned to worker 0
	int worker_id	///< identifier of current worker
	)
{
#ifdef microblaze
	return addr + to_even( 2 + (get_temp_step()+2) * worker_id );
#else
	UNUSED(worker_id);
	return addr;
#endif
}

static
int calc_and_set_temp_size(
	int temp_step)
{
#ifdef microblaze
	set_temp_step(temp_step);

	return 2 + to_even( 2 + (temp_step+2) * get_active_workers() );
#else
	// FIXME HACK: + 3*float => 16-align
	return temp_step + 3;
#endif
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

static
void *addr2(
	void *ptr,
	int y,
	int x,
	int stride_x,
	int stride_y)
{
	return (void *)((char *)ptr+y*stride_x+x*stride_y);
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

float *dwt_util_addr_coeff_s(
	void *ptr,
	int y,
	int x,
	int stride_x,
	int stride_y)
{
	return addr2_s(ptr, y, x, stride_x, stride_y);
}

double *dwt_util_addr_coeff_d(
	void *ptr,
	int y,
	int x,
	int stride_x,
	int stride_y)
{
	return addr2_d(ptr, y, x, stride_x, stride_y);
}

void *dwt_util_addr_coeff(
	void *ptr,
	int y,
	int x,
	int stride_x,
	int stride_y)
{
	return addr2(ptr, y, x, stride_x, stride_y);
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
		char *restrict ptr_dst = (char *restrict)dst;
		const char *restrict ptr_src = (const char *restrict)src;
		for(size_t i = 0; i < n; i++)
		{
			*(float *restrict)ptr_dst = *(const float *restrict)ptr_src;
	
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
		char *restrict ptr_dst = (char *restrict)dst;
		const char *restrict ptr_src = (const char *restrict)src;
		for(size_t i = 0; i < n; i++)
		{
			*(double *restrict)ptr_dst = *(const double *restrict)ptr_src;
	
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

void dwt_util_conv_show_s(
	const void *src,
	void *dst,
	int stride_x,
	int stride_y,
	int size_i_big_x,
	int size_i_big_y)
{
	assert( src != NULL && dst != NULL && size_i_big_x >= 0 && size_i_big_y >= 0 );

	const float a = 100.f;
	const float b = 10.f;

	for(int y = 0; y < size_i_big_y; y++)
	{
		for(int x = 0; x < size_i_big_x; x++)
		{
			const float coeff = *addr2_s(src, y, x, stride_x, stride_y);
			float *log_coeff = addr2_s(dst, y, x, stride_x, stride_y);

#ifdef microblaze
			*log_coeff = log(1.f+fabsf(coeff)*a)/b;
#else
			*log_coeff = logf(1.f+fabsf(coeff)*a)/b;
#endif
		}
	}
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
		float *arr_local = calc_temp_offset_s(arr, w);

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
void op4s_sdl2_import_preload_s_ref(float *out, const float *restrict addr)
{
	out[0] = addr[0];
	out[1] = addr[1];
	out[2] = addr[2];
	out[3] = addr[3];
}

static
void op4s_sdl2_import_s_ref(float *l, int idx, const float *out)
{
	l[idx] = out[idx];
}

static
void op4s_sdl6_import_s_ref(float *l, int idx, const float *out)
{
	l[idx] = out[idx];
}

static
void op4s_sdl2_load_s_ref(float *in, const float *addr)
{
	in[0] = addr[0];
	in[1] = addr[1];
	in[2] = addr[2];
	in[3] = addr[3];
}

static
void op4s_sdl2_shuffle_s_ref(float *c, float *r)
{
	c[0]=c[1]; c[1]=c[2]; c[2]=c[3];
	r[0]=r[1]; r[1]=r[2]; r[2]=r[3];
}

static
void op4s_sdl2_input_low_s_ref(const float *in, float *c, float *r)
{
	c[3] = in[0];
	r[3] = in[1];
}

static
void op4s_sdl2_input_high_s_ref(const float *in, float *c, float *r)
{
	c[3] = in[2];
	r[3] = in[3];
}

static
void op4s_sdl2_shuffle_input_low_s_ref(const float *in, float *c, float *r)
{
	op4s_sdl2_shuffle_s_ref(c, r);
	op4s_sdl2_input_low_s_ref(in, c, r);
}

static
void op4s_sdl2_shuffle_input_high_s_ref(const float *in, float *c, float *r)
{
	op4s_sdl2_shuffle_s_ref(c, r);
	op4s_sdl2_input_high_s_ref(in, c, r);
}

static
void op4s_sdl2_op_s_ref(float *z, const float *c, const float *w, const float *l, const float *r)
{
	z[3] = c[3] + w[3] * ( l[3] + r[3] );
	z[2] = c[2] + w[2] * ( l[2] + r[2] );
	z[1] = c[1] + w[1] * ( l[1] + r[1] );
	z[0] = c[0] + w[0] * ( l[0] + r[0] );
}

static
void op4s_sdl2_update_s_ref(float *c, float *l, float *r, const float *z)
{
	c[0] = l[0];
	c[1] = l[1];
	c[2] = l[2];
	c[3] = l[3];

	l[0] = r[0];
	l[1] = r[1];
	l[2] = r[2];
	l[3] = r[3];

	r[0] = z[0];
	r[1] = z[1];
	r[2] = z[2];
	r[3] = z[3];
}

static
void op4s_sdl6_update_s_ref(float *z, float *l, float *r)
{
	float t[4];

	t[0] = z[0];
	t[1] = z[1];
	t[2] = z[2];
	t[3] = z[3];

	z[0] = l[0];
	z[1] = l[1];
	z[2] = l[2];
	z[3] = l[3];

	l[0] = r[0];
	l[1] = r[1];
	l[2] = r[2];
	l[3] = r[3];

	r[0] = t[0];
	r[1] = t[1];
	r[2] = t[2];
	r[3] = t[3];
}

static
void op4s_sdl2_output_low_s_ref(float *out, const float *l, const float *z)
{
	out[0] = l[0];
	out[1] = z[0];
}

static
void op4s_sdl2_output_high_s_ref(float *out, const float *l, const float *z)
{
	out[2] = l[0];
	out[3] = z[0];
}

static
void op4s_sdl2_scale_s_ref(float *out, const float *v)
{
	out[0] *= v[0];
	out[1] *= v[1];
	out[2] *= v[2];
	out[3] *= v[3];
}

static
void op4s_sdl2_descale_s_ref(float *in, const float *v)
{
	in[0] *= v[0];
	in[1] *= v[1];
	in[2] *= v[2];
	in[3] *= v[3];
}

static
void op4s_sdl2_save_s_ref(float *out, float *addr)
{
	addr[0] = out[0];
	addr[1] = out[1];
}

static
void op4s_sdl2_save_shift_s_ref(float *out, float *addr)
{
	addr[0] = out[0];
	addr[1] = out[1];
	addr[2] = out[2];
	addr[3] = out[3];
}

static
void op4s_sdl2_export_s_ref(const float *l, float *addr, int idx)
{
	addr[idx] = l[idx];
}

static
void op4s_sdl6_export_s_ref(const float *l, float *addr, int idx)
{
	addr[idx] = l[idx];
}

static
void op4s_sdl_import_s_ref(float *l, const float *restrict addr, int idx)
{
	l[idx] = addr[idx];
}

static
void op4s_sdl_shuffle_s_ref(float *c, float *r)
{
	c[0]=c[1]; c[1]=c[2]; c[2]=c[3];
	r[0]=r[1]; r[1]=r[2]; r[2]=r[3];
}

static
void op4s_sdl_load_s_ref(float *in, const float *restrict addr)
{
	in[0] = addr[0];
	in[1] = addr[1];
}

static
void op4s_sdl_input_s_ref(const float *in, float *c, float *r)
{
	c[3] = in[0];
	r[3] = in[1];
}

static
void op4s_sdl_op_s_ref(float *z, const float *c, const float *w, const float *l, const float *r)
{
	z[3] = c[3] + w[3] * ( l[3] + r[3] );
	z[2] = c[2] + w[2] * ( l[2] + r[2] );
	z[1] = c[1] + w[1] * ( l[1] + r[1] );
	z[0] = c[0] + w[0] * ( l[0] + r[0] );
}

static
void op4s_sdl_update_s_ref(float *c, float *l, float *r, const float *z)
{
	c[0] = l[0];
	c[1] = l[1];
	c[2] = l[2];
	c[3] = l[3];

	l[0] = r[0];
	l[1] = r[1];
	l[2] = r[2];
	l[3] = r[3];

	r[0] = z[0];
	r[1] = z[1];
	r[2] = z[2];
	r[3] = z[3];
}

static
void op4s_sdl_output_s_ref(float *out, const float *l, const float *z)
{
	out[0] = l[0];
	out[1] = z[0];
}

static
void op4s_sdl_scale_s_ref(float *out, const float *v)
{
	out[0] *= v[0];
	out[1] *= v[1];
}

static
void op4s_sdl_descale_s_ref(float *in, const float *v)
{
	in[0] *= v[0];
	in[1] *= v[1];
}

static
void op4s_sdl_save_s_ref(float *out, float *restrict addr)
{
	addr[0] = out[0];
	addr[1] = out[1];
}

static
void op4s_sdl_export_s_ref(const float *l, float *restrict addr, int idx)
{
	addr[idx] = l[idx];
}

static
void op4s_sdl2_preload_fwd_prolog_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(w);
	UNUSED(v);
	UNUSED(l);
	UNUSED(c);
	UNUSED(r);
	UNUSED(z);
	UNUSED(in);

	op4s_sdl2_import_preload_s_ref(out, (*addr));

 	(*addr) += 4;
}

static
void op4s_sdl2_preload_inv_prolog_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(w);
	UNUSED(v);
	UNUSED(l);
	UNUSED(c);
	UNUSED(r);
	UNUSED(z);
	UNUSED(in);

	op4s_sdl2_import_preload_s_ref(out, (*addr));

 	(*addr) += 4;
}

static
void op4s_sdl6_preload_prolog_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(w);
	UNUSED(v);
	UNUSED(l);
	UNUSED(r);
	UNUSED(z);
	UNUSED(in);

	op4s_sdl2_import_preload_s_ref(out, (*addr));

 	(*addr) += 4;
}

static
void op4s_sdl2_pass_fwd_prolog_full_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(out);

	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl2_pass_inv_prolog_full_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(out);

	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// descale
	op4s_sdl2_descale_s_ref(in, v);

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl6_pass_inv_prolog_full_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(out);

	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// descale
	op4s_sdl2_descale_s_ref(in, v);

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// update
	op4s_sdl6_update_s_ref(z, l, r);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl6_pass_fwd_prolog_full_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(out);

	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// (descale)

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// update
	op4s_sdl6_update_s_ref(z, l, r);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl2_pass_fwd_prolog_light_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(out);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);
}

static
void op4s_sdl2_pass_inv_prolog_light_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(out);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);
}

static
void op4s_sdl6_pass_inv_prolog_light_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(out);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// update
	op4s_sdl6_update_s_ref(z, l, r);
}

static
void op4s_sdl6_pass_fwd_prolog_light_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(out);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// update
	op4s_sdl6_update_s_ref(z, l, r);
}

static
void op4s_sdl2_pass_fwd_core_light_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);
}

static
void op4s_sdl2_pass_inv_core_light_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);
}

static
void op4s_sdl6_pass_inv_core_light_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// (update)
}

static
void op4s_sdl6_pass_fwd_core_light_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// (update)
}

static
void op4s_sdl6_pass_inv_postcore_light_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// update
	op4s_sdl6_update_s_ref(z, l, r);
}

static
void op4s_sdl6_pass_fwd_postcore_light_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(addr);

	// shuffle + input-high
	op4s_sdl2_shuffle_input_high_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// update
	op4s_sdl6_update_s_ref(z, l, r);
}

static
void op4s_sdl2_pass_fwd_core_full_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// scale
	op4s_sdl2_scale_s_ref(out, v);

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl2_pass_inv_core_full_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// descale
	op4s_sdl2_descale_s_ref(in, v);

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// (scale)

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl6_pass_inv_core_full_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// descale
	op4s_sdl2_descale_s_ref(in, v);

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// (scale)

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// (update)

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl6_pass_fwd_core_full_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// (descale)

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// (scale)
	op4s_sdl2_scale_s_ref(out, v);

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// (update)

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl6_pass_inv_postcore_full_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// descale
	op4s_sdl2_descale_s_ref(in, v);

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// (scale)

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// update
	op4s_sdl6_update_s_ref(z, l, r);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl6_pass_fwd_postcore_full_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	// load
	op4s_sdl2_load_s_ref(in, (*addr));

	// (descale)

	// shuffle + input-low
	op4s_sdl2_shuffle_input_low_s_ref(in, z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// (scale)
	op4s_sdl2_scale_s_ref(out, v);

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// update
	op4s_sdl6_update_s_ref(z, l, r);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl2_pass_fwd_epilog_full_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(in);

	// shuffle
	op4s_sdl2_shuffle_s_ref(c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// scale
	op4s_sdl2_scale_s_ref(out, v);

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl2_pass_inv_epilog_full_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);

	// shuffle
	op4s_sdl2_shuffle_s_ref(c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// (scale)

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl6_pass_inv_epilog_full_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);

	// shuffle
	op4s_sdl2_shuffle_s_ref(z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// (scale)

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// update
	op4s_sdl6_update_s_ref(z, l, r);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl6_pass_fwd_epilog_full_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);

	// shuffle
	op4s_sdl2_shuffle_s_ref(z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-high
	op4s_sdl2_output_high_s_ref(out, l, z);

	// (scale)
	op4s_sdl2_scale_s_ref(out, v);

	// save-shift
	op4s_sdl2_save_shift_s_ref(out, (*addr)-12);

	// update
	op4s_sdl6_update_s_ref(z, l, r);

	// pointers
	(*addr) += 4;
}

static
void op4s_sdl2_pass_fwd_epilog_light_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);
	UNUSED(addr);

	// shuffle
	op4s_sdl2_shuffle_s_ref(c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);
}

static
void op4s_sdl2_pass_inv_epilog_light_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);
	UNUSED(addr);

	// shuffle
	op4s_sdl2_shuffle_s_ref(c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);
}

static
void op4s_sdl6_pass_inv_epilog_light_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);
	UNUSED(addr);

	// shuffle
	op4s_sdl2_shuffle_s_ref(z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// update
	op4s_sdl6_update_s_ref(z, l, r);
}

static
void op4s_sdl6_pass_fwd_epilog_light_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);
	UNUSED(addr);

	// shuffle
	op4s_sdl2_shuffle_s_ref(z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// update
	op4s_sdl6_update_s_ref(z, l, r);
}

static
void op4s_sdl2_pass_fwd_epilog_flush_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(in);

	// shuffle
	op4s_sdl2_shuffle_s_ref(c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// scale
	op4s_sdl2_scale_s_ref(out, v);

	// save
	op4s_sdl2_save_s_ref(out, (*addr)-12);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);
}

static
void op4s_sdl2_pass_inv_epilog_flush_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);

	// shuffle
	op4s_sdl2_shuffle_s_ref(c, r);

	// operation
	op4s_sdl2_op_s_ref(z, c, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// (scale)

	// save
	op4s_sdl2_save_s_ref(out, (*addr)-12);

	// update
	op4s_sdl2_update_s_ref(c, l, r, z);
}

static
void op4s_sdl6_pass_inv_epilog_flush_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);

	// shuffle
	op4s_sdl2_shuffle_s_ref(z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// (scale)

	// save
	op4s_sdl2_save_s_ref(out, (*addr)-12);

	// update
	op4s_sdl6_update_s_ref(z, l, r);
}

static
void op4s_sdl6_pass_fwd_epilog_flush_s_ref(const float *w, const float *v, float *l, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);

	// shuffle
	op4s_sdl2_shuffle_s_ref(z, r);

	// operation
	op4s_sdl2_op_s_ref(z, z, w, l, r);

	// output-low
	op4s_sdl2_output_low_s_ref(out, l, z);

	// (scale)
	op4s_sdl2_scale_s_ref(out, v);

	// save
	op4s_sdl2_save_s_ref(out, (*addr)-12);

	// update
	op4s_sdl6_update_s_ref(z, l, r);
}

static
void op4s_sdl_pass_fwd_prolog_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(out);

	// shuffle
	op4s_sdl_shuffle_s_ref(c, r);

	// load
	op4s_sdl_load_s_ref(in, *addr+4);

	// (descale)

	// input
	op4s_sdl_input_s_ref(in, c, r);

	// operation
	op4s_sdl_op_s_ref(z, c, w, l, r);

	// (output)

	// (scale)

	// (save)

	// update
	op4s_sdl_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 2;
}

static
void op4s_sdl_pass_inv_prolog_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(out);

	// shuffle
	op4s_sdl_shuffle_s_ref(c, r);

	// load
	op4s_sdl_load_s_ref(in, *addr+4);

	// descale
	op4s_sdl_descale_s_ref(in, v);

	// input
	op4s_sdl_input_s_ref(in, c, r);

	// operation
	op4s_sdl_op_s_ref(z, c, w, l, r);

	// (output)

	// (scale)

	// (save)

	// update
	op4s_sdl_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 2;
}

static
void op4s_sdl_pass_fwd_core_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	// shuffle
	op4s_sdl_shuffle_s_ref(c, r);

	// load
	op4s_sdl_load_s_ref(in, *addr+4);

	// (descale)

	// input
	op4s_sdl_input_s_ref(in, c, r);

	// operation
	op4s_sdl_op_s_ref(z, c, w, l, r);

	// output
	op4s_sdl_output_s_ref(out, l, z);

	// scale
	op4s_sdl_scale_s_ref(out, v);

	// save
	op4s_sdl_save_s_ref(out, *addr-6);

	// update
	op4s_sdl_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 2;
}

static
void op4s_sdl_pass_inv_core_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	// shuffle
	op4s_sdl_shuffle_s_ref(c, r);

	// load
	op4s_sdl_load_s_ref(in, *addr+4);

	// descale
	op4s_sdl_descale_s_ref(in, v);

	// input
	op4s_sdl_input_s_ref(in, c, r);

	// operation
	op4s_sdl_op_s_ref(z, c, w, l, r);

	// output
	op4s_sdl_output_s_ref(out, l, z);

	// (scale)

	// save
	op4s_sdl_save_s_ref(out, *addr-6);

	// update
	op4s_sdl_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 2;
}

static
void op4s_sdl_pass_fwd_epilog_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(in);

	// shuffle
	op4s_sdl_shuffle_s_ref(c, r);

	// (load)

	// (descale)

	// (input)

	// operation
	op4s_sdl_op_s_ref(z, c, w, l, r);

	// output
	op4s_sdl_output_s_ref(out, l, z);

	// scale
	op4s_sdl_scale_s_ref(out, v);

	// save
	op4s_sdl_save_s_ref(out, *addr-6);

	// update
	op4s_sdl_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 2;
}

static
void op4s_sdl_pass_inv_epilog_s_ref(const float *w, const float *v, float *l, float *c, float *r, float *z, float *in, float *out, float *restrict *addr)
{
	UNUSED(v);
	UNUSED(in);

	// shuffle
	op4s_sdl_shuffle_s_ref(c, r);

	// (load)

	// (descale)

	// (input)

	// operation
	op4s_sdl_op_s_ref(z, c, w, l, r);

	// (output)
	op4s_sdl_output_s_ref(out, l, z);

	// (scale)

	// save
	op4s_sdl_save_s_ref(out, *addr-6);

	// update
	op4s_sdl_update_s_ref(c, l, r, z);

	// pointers
	(*addr) += 2;
}

// 6 iterations merger, i.e. 12=2*(3*2) coefficient per iteration
static
void accel_lift_op4s_main_sdl6_ref_s(
	float *restrict arr,
	int steps,
	float alpha,
	float beta,
	float gamma,
	float delta,
	float zeta,
	int scaling)
{
	// 6+ coeffs implies 3+ steps
	assert( steps >= 3 );

	assert( is_aligned_16(arr) );

	const float w[4] = { delta, gamma, beta, alpha };
	const float v[4] = { 1/zeta, zeta, 1/zeta, zeta };
	float l[4];
	float r[4];
	float z[4];
	float in[4];
	float out[4];

	const int S = steps-3;
	const int U = S / 6;
	const int M = S % 6;
	const int T = M >> 1;

	if( scaling < 0 )
	{
		// ****** inverse transform ******

		// FIXME(ASVP): support for several workers
		assert( 1 == dwt_util_get_num_workers() );

		// *** init ***

		float *restrict addr = arr;

		// *** prolog2 ***

		// prolog2: import-preload
		op4s_sdl6_preload_prolog_s_ref(w, v, l, r, z, in, out, &addr);

		// prolog2: import(3)
		op4s_sdl6_import_s_ref(l, 3, out);

		// prolog2: pass-prolog-full
		op4s_sdl6_pass_inv_prolog_full_s_ref(w, v, l, r, z, in, out, &addr);

		// prolog2: import(2)
		op4s_sdl6_import_s_ref(l, 2, out);

		// prolog2: pass-prolog-light
		op4s_sdl6_pass_inv_prolog_light_s_ref(w, v, l, r, z, in, out, &addr);

		// prolog2: import(1)
		op4s_sdl6_import_s_ref(l, 1, out);

		// prolog2: pass-prolog-full
		op4s_sdl6_pass_inv_prolog_full_s_ref(w, v, l, r, z, in, out, &addr);

		// prolog2: import(0)
		op4s_sdl6_import_s_ref(l, 0, out);

		// *** core ***

		// core: for u = 0 to U
		for(int u = 0; u < U; u++)
		{
			// NOTE: l, r, z

			// core: pass1-core-light
			op4s_sdl6_pass_inv_core_light_s_ref(w, v, /*l*/l, /*r*/r, /*z*/z, in, out, &addr);

			// NOTE: z => l, l => r, r => z -- 1st rotation

			// core: pass1-core-full
			op4s_sdl6_pass_inv_core_full_s_ref(w, v, /*l*/r, /*r*/z, /*z*/l, in, out, &addr);

			// NOTE: (r => z) => l, (z => l) => r, (l => r) => z -- 2nd rotation

			// core: pass2-core-light
			op4s_sdl6_pass_inv_core_light_s_ref(w, v, /*l*/z, /*r*/l, /*z*/r, in, out, &addr);

			// NOTE: ((l => r) => z) => l, ((r => z) => l) => r, ((z => l) => r) => z -- 3rd rotation, as at the beginning

			// core: pass2-core-full
			op4s_sdl6_pass_inv_core_full_s_ref(w, v, /*l*/l, /*r*/r, /*z*/z, in, out, &addr);

			// NOTE: z => l, l => r, r => z -- 1st rotation

			// core: pass3-core-light
			op4s_sdl6_pass_inv_core_light_s_ref(w, v, /*l*/r, /*r*/z, /*z*/l, in, out, &addr);

			// NOTE: (r => z) => l, (z => l) => r, (l => r) => z -- 2nd rotation

			// core: pass3-core-full
			op4s_sdl6_pass_inv_core_full_s_ref(w, v, /*l*/z, /*r*/l, /*z*/r, in, out, &addr);

			// NOTE: ((l => r) => z) => l, ((r => z) => l) => r, ((z => l) => r) => z -- 3rd rotation, as at the beginning
		}

		// core: for t = 0 to T do
		for(int t = 0; t < T; t++)
		{
			// core: pass-core-light
			op4s_sdl6_pass_inv_postcore_light_s_ref(w, v, l, r, z, in, out, &addr);

			// core: pass-core-full
			op4s_sdl6_pass_inv_postcore_full_s_ref(w, v, l, r, z, in, out, &addr);
		}

		// core: if odd then
		if( is_odd(S) )
		{
			// core: pass-core-light
			op4s_sdl6_pass_inv_postcore_light_s_ref(w, v, l, r, z, in, out, &addr);
		}

		// *** epilog2 ***

		if( is_odd(S) )
		{
			// epilog2: export(3)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 3);

			// epilog2: pass-epilog-full
			op4s_sdl6_pass_inv_epilog_full_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(2)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 2);

			// epilog2: pass-epilog-light
			op4s_sdl6_pass_inv_epilog_light_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(1)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 1);

			// epilog2: pass-epilog-full
			op4s_sdl6_pass_inv_epilog_full_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(0)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 0);
		}
		else
		{
			// epilog2: export(3)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 3);

			// epilog2: pass-epilog-light
			op4s_sdl6_pass_inv_epilog_light_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(2)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 2);

			// epilog2: pass-epilog-full
			op4s_sdl6_pass_inv_epilog_full_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(1)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 1);

			// epilog2: pass-epilog-flush
			op4s_sdl6_pass_inv_epilog_flush_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(0)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 0);
		}
	}
	else if ( scaling > 0 )
	{
		// ****** forward transform ******

		// FIXME(ASVP): support for several workers
		assert( 1 == dwt_util_get_num_workers() );

		// *** init ***

		float *restrict addr = arr;

		// *** prolog2 ***

		// prolog2: import-preload
		op4s_sdl6_preload_prolog_s_ref(w, v, l, r, z, in, out, &addr);

		// prolog2: import(3)
		op4s_sdl6_import_s_ref(l, 3, out);

		// prolog2: pass-prolog-full
		op4s_sdl6_pass_fwd_prolog_full_s_ref(w, v, l, r, z, in, out, &addr);

		// prolog2: import(2)
		op4s_sdl6_import_s_ref(l, 2, out);

		// prolog2: pass-prolog-light
		op4s_sdl6_pass_fwd_prolog_light_s_ref(w, v, l, r, z, in, out, &addr);

		// prolog2: import(1)
		op4s_sdl6_import_s_ref(l, 1, out);

		// prolog2: pass-prolog-full
		op4s_sdl6_pass_fwd_prolog_full_s_ref(w, v, l, r, z, in, out, &addr);

		// prolog2: import(0)
		op4s_sdl6_import_s_ref(l, 0, out);

		// *** core ***

		// core: for u = 0 to U
		for(int u = 0; u < U; u++)
		{
			// NOTE: l, r, z

			// core: pass1-core-light
			op4s_sdl6_pass_fwd_core_light_s_ref(w, v, /*l*/l, /*r*/r, /*z*/z, in, out, &addr);

			// NOTE: z => l, l => r, r => z

			// core: pass1-core-full
			op4s_sdl6_pass_fwd_core_full_s_ref(w, v, /*l*/r, /*r*/z, /*z*/l, in, out, &addr);

			// NOTE: (r => z) => l, (z => l) => r, (l => r) => z

			// core: pass2-core-light
			op4s_sdl6_pass_fwd_core_light_s_ref(w, v, /*l*/z, /*r*/l, /*z*/r, in, out, &addr);

			// NOTE: ((l => r) => z) => l, ((r => z) => l) => r, ((z => l) => r)

			// core: pass2-core-full
			op4s_sdl6_pass_fwd_core_full_s_ref(w, v, /*l*/l, /*r*/r, /*z*/z, in, out, &addr);

			// NOTE: z => l, l => r, r => z

			// core: pass3-core-light
			op4s_sdl6_pass_fwd_core_light_s_ref(w, v, /*l*/r, /*r*/z, /*z*/l, in, out, &addr);

			// NOTE: (r => z) => l, (z => l) => r, (l => r) => z

			// core: pass3-core-full
			op4s_sdl6_pass_fwd_core_full_s_ref(w, v, /*l*/z, /*r*/l, /*z*/r, in, out, &addr);

			// NOTE: ((l => r) => z) => l, ((r => z) => l) => r, ((z => l) => r) => z
		}

		// core: for t = 0 to T do
		for(int t = 0; t < T; t++)
		{
			// core: pass-core-light
			op4s_sdl6_pass_fwd_postcore_light_s_ref(w, v, l, r, z, in, out, &addr);

			// core: pass-core-full
			op4s_sdl6_pass_fwd_postcore_full_s_ref(w, v, l, r, z, in, out, &addr);
		}

		// core: if odd then
		if( is_odd(S) )
		{
			// core: pass-core-light
			op4s_sdl6_pass_fwd_postcore_light_s_ref(w, v, l, r, z, in, out, &addr);
		}

		// *** epilog2 ***

		if( is_odd(S) )
		{
			// epilog2: export(3)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 3);

			// epilog2: pass-epilog-full
			op4s_sdl6_pass_fwd_epilog_full_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(2)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 2);

			// epilog2: pass-epilog-light
			op4s_sdl6_pass_fwd_epilog_light_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(1)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 1);

			// epilog2: pass-epilog-full
			op4s_sdl6_pass_fwd_epilog_full_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(0)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 0);
		}
		else
		{
			// epilog2: export(3)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 3);

			// epilog2: pass-epilog-light
			op4s_sdl6_pass_fwd_epilog_light_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(2)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 2);

			// epilog2: pass-epilog-full
			op4s_sdl6_pass_fwd_epilog_full_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(1)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 1);

			// epilog2: pass-epilog-flush
			op4s_sdl6_pass_fwd_epilog_flush_s_ref(w, v, l, r, z, in, out, &addr);

			// epilog2: export(0)
			op4s_sdl6_export_s_ref(l, &arr[2*steps], 0);
		}
	}
	else
	{
		// ****** transform w/o scaling ******

		// not implemented yet
		dwt_util_abort();
	}
}

// 2 iterations merged, i.e. loads 4 (2*2) coeffs in every iter.
static
void accel_lift_op4s_main_sdl2_ref_s(
	float *restrict arr,
	int steps,
	float alpha,
	float beta,
	float gamma,
	float delta,
	float zeta,
	int scaling)
{
	// 6+ coeffs implies 3+ steps
	assert( steps >= 3 );

	assert( is_aligned_16(arr) );

	const float w[4] = { delta, gamma, beta, alpha };
	const float v[4] = { 1/zeta, zeta, 1/zeta, zeta };
	float l[4];
	float c[4];
	float r[4];
	float z[4];
	float in[4];
	float out[4];

	const int S = steps-3;
	const int T = S >> 1;

	if( scaling < 0 )
	{
		// ****** inverse transform ******

		// FIXME(ASVP): support for several workers
		assert( 1 == dwt_util_get_num_workers() );

		// *** init ***

		float *restrict addr = arr;

		// *** prolog2 ***

		// prolog2: import-preload
		op4s_sdl2_preload_inv_prolog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(3)
		op4s_sdl2_import_s_ref(l, 3, out);

		// prolog2: pass-prolog-full
		op4s_sdl2_pass_inv_prolog_full_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(2)
		op4s_sdl2_import_s_ref(l, 2, out);

		// prolog2: pass-prolog-light
		op4s_sdl2_pass_inv_prolog_light_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(1)
		op4s_sdl2_import_s_ref(l, 1, out);

		// prolog2: pass-prolog-full
		op4s_sdl2_pass_inv_prolog_full_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(0)
		op4s_sdl2_import_s_ref(l, 0, out);

		// *** core ***

		// core: for t = 0 to T do
		for(int t = 0; t < T; t++)
		{
			// core: pass-core-light
			op4s_sdl2_pass_inv_core_light_s_ref(w, v, l, c, r, z, in, out, &addr);

			// core: pass-core-full
			op4s_sdl2_pass_inv_core_full_s_ref(w, v, l, c, r, z, in, out, &addr);
		}

		// core: if odd then
		if( is_odd(S) )
		{
			// core: pass-core-light
			op4s_sdl2_pass_inv_core_light_s_ref(w, v, l, c, r, z, in, out, &addr);
		}

		// *** epilog2 ***

		if( is_odd(S) )
		{
			// epilog2: export(3)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 3);

			// epilog2: pass-epilog-full
			op4s_sdl2_pass_inv_epilog_full_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(2)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 2);

			// epilog2: pass-epilog-light
			op4s_sdl2_pass_inv_epilog_light_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(1)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 1);

			// epilog2: pass-epilog-full
			op4s_sdl2_pass_inv_epilog_full_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(0)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 0);
		}
		else
		{
			// epilog2: export(3)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 3);

			// epilog2: pass-epilog-light
			op4s_sdl2_pass_inv_epilog_light_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(2)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 2);

			// epilog2: pass-epilog-full
			op4s_sdl2_pass_inv_epilog_full_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(1)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 1);

			// epilog2: pass-epilog-flush
			op4s_sdl2_pass_inv_epilog_flush_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(0)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 0);
		}
	}
	else if ( scaling > 0 )
	{
		// ****** forward transform ******

		// FIXME(ASVP): support for several workers
		assert( 1 == dwt_util_get_num_workers() );

		// *** init ***

		float *restrict addr = arr;

		// *** prolog2 ***

		// prolog2: import-preload
		op4s_sdl2_preload_fwd_prolog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(3)
		op4s_sdl2_import_s_ref(l, 3, out);

		// prolog2: pass-prolog-full
		op4s_sdl2_pass_fwd_prolog_full_s_ref(w, v, l, c, r, z, in, out, &addr);
		const int S = steps-3;
		const int T = S >> 1;

		// prolog2: import(2)
		op4s_sdl2_import_s_ref(l, 2, out);

		// prolog2: pass-prolog-light
		op4s_sdl2_pass_fwd_prolog_light_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(1)
		op4s_sdl2_import_s_ref(l, 1, out);

		// prolog2: pass-prolog-full
		op4s_sdl2_pass_fwd_prolog_full_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(0)
		op4s_sdl2_import_s_ref(l, 0, out);

		// *** core ***

		// core: for t = 0 to T do
		for(int t = 0; t < T; t++)
		{
			// core: pass-core-light
			op4s_sdl2_pass_fwd_core_light_s_ref(w, v, l, c, r, z, in, out, &addr);

			// core: pass-core-full
			op4s_sdl2_pass_fwd_core_full_s_ref(w, v, l, c, r, z, in, out, &addr);
		}

		// core: if odd then
		if( is_odd(S) )
		{
			// core: pass-core-light
			op4s_sdl2_pass_fwd_core_light_s_ref(w, v, l, c, r, z, in, out, &addr);
		}

		// *** epilog2 ***

		if( is_odd(S) )
		{
			// epilog2: export(3)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 3);

			// epilog2: pass-epilog-full
			op4s_sdl2_pass_fwd_epilog_full_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(2)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 2);

			// epilog2: pass-epilog-light
			op4s_sdl2_pass_fwd_epilog_light_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(1)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 1);

			// epilog2: pass-epilog-full
			op4s_sdl2_pass_fwd_epilog_full_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(0)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 0);
		}
		else
		{
			// epilog2: export(3)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 3);

			// epilog2: pass-epilog-light
			op4s_sdl2_pass_fwd_epilog_light_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(2)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 2);

			// epilog2: pass-epilog-full
			op4s_sdl2_pass_fwd_epilog_full_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(1)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 1);

			// epilog2: pass-epilog-flush
			op4s_sdl2_pass_fwd_epilog_flush_s_ref(w, v, l, c, r, z, in, out, &addr);

			// epilog2: export(0)
			op4s_sdl2_export_s_ref(l, &arr[2*steps], 0);
		}
	}
	else
	{
		// ****** transform w/o scaling ******

		// not implemented yet
		dwt_util_abort();
	}
}

static
void accel_lift_op4s_main_sdl_ref_s(
	float *restrict arr,
	int steps,
	float alpha,
	float beta,
	float gamma,
	float delta,
	float zeta,
	int scaling)
{
	// 6+ coeffs implies 3+ steps
	assert( steps >= 3 );

	assert( is_aligned_16(arr) );

	const float w[4] = { delta, gamma, beta, alpha };
	const float v[4] = { 1/zeta, zeta, 1/zeta, zeta };
	float l[4];
	float c[4];
	float r[4];
	float z[4];
	float in[4];
	float out[4];

	const int S = steps-3;

	if( scaling < 0 )
	{
		// ****** inverse transform ******

		// FIXME(ASVP): support for several workers
		assert( 1 == dwt_util_get_num_workers() );

		// *** init ***

		// this pointer is needed because of use of arr[] in import and export functions
		float *restrict addr = arr;

		// *** prolog2 ***

		// prolog2: import(3)
		op4s_sdl_import_s_ref(l, arr, 3);

		// prolog2: pass-prolog
		op4s_sdl_pass_inv_prolog_s_ref(w, v, l, c, r, z, in, out, &addr);
	
		// prolog2: import(2)
		op4s_sdl_import_s_ref(l, arr, 2);

		// prolog2: pass-prolog
		op4s_sdl_pass_inv_prolog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(1)
		op4s_sdl_import_s_ref(l, arr, 1);

		// prolog2: pass-prolog
		op4s_sdl_pass_inv_prolog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(0)
		op4s_sdl_import_s_ref(l, arr, 0);

		// *** core ***

		// core: for s = 0 to S do
		for(int s = 0; s < S; s++)
		{
			// core: pass-core
			op4s_sdl_pass_inv_core_s_ref(w, v, l, c, r, z, in, out, &addr);

		}

		// *** epilog2 ***

		// epilog2: export(3)
		op4s_sdl_export_s_ref(l, &arr[2*steps], 3);

		// epilog2: pass-epilog
		op4s_sdl_pass_inv_epilog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// epilog2: export(2)
		op4s_sdl_export_s_ref(l, &arr[2*steps], 2);

		// epilog2: pass-epilog
		op4s_sdl_pass_inv_epilog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// epilog2: export(1)
		op4s_sdl_export_s_ref(l, &arr[2*steps], 1);

		// epilog2: pass-epilog
		op4s_sdl_pass_inv_epilog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// epilog2: export(0)
		op4s_sdl_export_s_ref(l, &arr[2*steps], 0);
	}
	else if ( scaling > 0 )
	{
		// ****** forward transform ******

		// FIXME(ASVP): support for several workers
		assert( 1 == dwt_util_get_num_workers() );

		// *** init ***

		// this pointer is needed because of use of arr[] in import and export functions
		float *restrict addr = arr;

		// *** prolog2 ***

		// prolog2: import(3)
		op4s_sdl_import_s_ref(l, arr, 3);

		// prolog2: pass-prolog
		op4s_sdl_pass_fwd_prolog_s_ref(w, v, l, c, r, z, in, out, &addr);
	
		// prolog2: import(2)
		op4s_sdl_import_s_ref(l, arr, 2);

		// prolog2: pass-prolog
		op4s_sdl_pass_fwd_prolog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(1)
		op4s_sdl_import_s_ref(l, arr, 1);

		// prolog2: pass-prolog
		op4s_sdl_pass_fwd_prolog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// prolog2: import(0)
		op4s_sdl_import_s_ref(l, arr, 0);

		// *** core ***

		// core: for s = 0 to S-3 do
		for(int s = 0; s < S; s++)
		{
			// NOTE: core: pass-core
			op4s_sdl_pass_fwd_core_s_ref(w, v, l, c, r, z, in, out, &addr);

		}

		// *** epilog2 ***

		// epilog2: export(3)
		op4s_sdl_export_s_ref(l, &arr[2*steps], 3);

		// epilog2: pass-epilog
		op4s_sdl_pass_fwd_epilog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// epilog2: export(2)
		op4s_sdl_export_s_ref(l, &arr[2*steps], 2);

		// epilog2: pass-epilog
		op4s_sdl_pass_fwd_epilog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// epilog2: export(1)
		op4s_sdl_export_s_ref(l, &arr[2*steps], 1);

		// epilog2: pass-epilog
		op4s_sdl_pass_fwd_epilog_s_ref(w, v, l, c, r, z, in, out, &addr);

		// epilog2: export(0)
		op4s_sdl_export_s_ref(l, &arr[2*steps], 0);
	}
	else
	{
		// ****** transform w/o scaling ******

		// not implemented yet
		dwt_util_abort();
	}
}

/**
 * "double-loop algorithm" from Rade Kutil: A Single-Loop Approach to SIMD Parallelization of 2-D Wavelet Lifting.
 */
static
void accel_lift_op4s_main_dl_s(
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

	if( scaling < 0 )
	{
		for(int w = 0; w < dwt_util_get_num_workers(); w++)
		{
			float *arr_local = calc_temp_offset_s(arr, w);

			const float w[4] = { delta, gamma, beta, alpha };

			// values that have to be passed from iteration to iteration
			// slide in left border
			float l[4] = { arr_local[0], arr_local[1], arr_local[2], arr_local[3] };

			// loop by pairs from left to right
			for(int s = 0; s < steps; s++)
			{
				// auxiliary variables
				float in0;
				float in1;
				float out0;
				float out1;

				// inputs
				in0 = arr_local[4+0+s*2];
				in1 = arr_local[4+1+s*2];

				// scales
				in0 = in0 * 1/zeta;
				in1 = in1 *   zeta;

				// shuffles
				float c[4] = { l[1], l[2], l[3], in0 };
				out0 = l[0];

				float r[4];

				// operation z[] = c[] + { alpha, beta, gamma, delta } * ( l[] + r[] )
				// by sequential computation from top/right to bottom/left
				r[3] = in1;
				r[2] = c[3]+w[3]*(l[3]+r[3]);
				r[1] = c[2]+w[2]*(l[2]+r[2]);
				r[0] = c[1]+w[1]*(l[1]+r[1]);
				out1 = c[0]+w[0]*(l[0]+r[0]);

				// outputs
				arr_local[0+0+s*2] = out0;
				arr_local[0+1+s*2] = out1;

				// update l[]
				l[0] = r[0];
				l[1] = r[1];
				l[2] = r[2];
				l[3] = r[3];
			}

			// slide out right border
			arr_local[steps*2+0] = l[0];
			arr_local[steps*2+1] = l[1];
			arr_local[steps*2+2] = l[2];
			arr_local[steps*2+3] = l[3];
		}
	}
	else if ( scaling > 0 )
	{
		for(int w = 0; w < dwt_util_get_num_workers(); w++)
		{
			float *arr_local = calc_temp_offset_s(arr, w);

			const float w[4] = { delta, gamma, beta, alpha };

			// values that have to be passed from iteration to iteration
			// slide in left border
			float l[4] = { arr_local[0], arr_local[1], arr_local[2], arr_local[3] };

			// loop by pairs from left to right
			for(int s = 0; s < steps; s++)
			{
				// auxiliary variables
				float in0;
				float in1;
				float out0;
				float out1;

				// inputs
				in0 = arr_local[4+0+s*2];
				in1 = arr_local[4+1+s*2];

				// shuffles
				float c[4] = { l[1], l[2], l[3], in0 };
				out0 = l[0];

				float r[4];

				// operation z[] = c[] + { alpha, beta, gamma, delta } * ( l[] + r[] )
				// by sequential computation from top/right to bottom/left
				r[3] = in1;
				r[2] = c[3]+w[3]*(l[3]+r[3]);
				r[1] = c[2]+w[2]*(l[2]+r[2]);
				r[0] = c[1]+w[1]*(l[1]+r[1]);
				out1 = c[0]+w[0]*(l[0]+r[0]);

				// scales
				out0 = out0 * 1/zeta;
				out1 = out1 *   zeta;

				// outputs
				arr_local[0+0+s*2] = out0;
				arr_local[0+1+s*2] = out1;

				// update l[]
				l[0] = r[0];
				l[1] = r[1];
				l[2] = r[2];
				l[3] = r[3];
			}

			// slide out right border
			arr_local[steps*2+0] = l[0];
			arr_local[steps*2+1] = l[1];
			arr_local[steps*2+2] = l[2];
			arr_local[steps*2+3] = l[3];
		}
	}
	else
	{
		// fallback, not implemented
		accel_lift_op4s_main_s(arr, steps, alpha, beta, gamma, delta, zeta, scaling);
	}
}

int dwt_util_is_aligned_8(
	const void *ptr)
{
	return is_aligned_8(ptr);
}

int dwt_util_is_aligned_4(
	const void *ptr)
{
	return is_aligned_4(ptr);
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

	assert( steps <= (BANK_SIZE - 4) / 2 );

	const int size = 2*steps + 4;
	float *addr = arr; // FIXME: not needed

	assert( is_aligned_8(addr) );
	assert( is_even(size) );

	for(int w = 0; w < get_active_workers(); w++)
	{
		// FIXME: channel w according to worker ID; but each worker has independent DMA channels, thus this is not necessary
		const uint8_t ch = w;
		float *addr_local = calc_temp_offset_s(addr, w);
		
		assert( is_aligned_8(addr_local) );

		WAL_CHECK( wal_dma_configure(worker[w], ch, addr_local, 0, WAL_BCE_JSY_DMEM_A, WAL_BANK_POS(0), size) );
		WAL_CHECK( wal_dma_start(worker[w], ch, WAL_DMA_REQ_RD) );
	}

	for(int w = 0; w < get_active_workers(); w++)
	{
		// HACK: wait for completing memory transfers on all 8 channels; but each worker has independent DMA channels
		while( wal_dma_isbusy(worker[w], /*WAL_DMA_MASK(ch)*/ 0x0f) )
			;
	}

	const uint32_t steps_32 = (uint32_t)steps;

	// start BCE computations
	for(int w = 0; w < get_active_workers(); w++)
	{
		WAL_CHECK( wal_mb2cmem(worker[w], WAL_CMEM_MB2PB, 0x01, &steps_32, 1) );

		WAL_CHECK( wal_mb2pb(worker[w], 1) );
	}

	// wait for finishing every BCE computation
	for(int w = 0; w < get_active_workers(); w++)
	{
		WAL_CHECK( wal_pb2mb(worker[w], NULL) );
	}

	assert( is_aligned_8(addr) );
	assert( is_even(size) );

	for(int w = 0; w < get_active_workers(); w++)
	{
		const uint8_t ch = w;
		float *addr_local = calc_temp_offset_s(addr, w);

		assert( is_aligned_8(addr_local) );

		WAL_CHECK( wal_dma_configure(worker[w], ch, addr_local, 0, WAL_BCE_JSY_DMEM_C, WAL_BANK_POS(0), size) );
		WAL_CHECK( wal_dma_start(worker[w], ch, WAL_DMA_REQ_WR) );
	}

	for(int w = 0; w < get_active_workers(); w++)
	{
		while( wal_dma_isbusy(worker[w], /*WAL_DMA_MASK(ch)*/ 0x0f) )
			;
	}

	for(int w = 0; w < get_active_workers(); w++)
	{
		float *addr_local = calc_temp_offset_s(addr, w);

		flush_cache_s(addr_local-1, size); // HACK: why -1?
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
		float *arr_local = calc_temp_offset_s(arr, w);
		
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
		float *arr_local = calc_temp_offset_s(arr, w);

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
		float *arr_local = calc_temp_offset_s(arr, w);

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
	float *restrict arr,
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

		// FIXME: with GCC use (un)likely, i.e. __builtin_expect
		if(1 == get_accel_type())
		{
			const int max_inner_len = to_even(BANK_SIZE) - 4;
			const int inner_len = to_even(len-off) - 4;
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
				if( steps > 25 )
					accel_lift_op4s_main_pb_s(&arr[left], steps, alpha, beta, gamma, delta, zeta, scaling);
				else
					accel_lift_op4s_main_s(&arr[left], steps, alpha, beta, gamma, delta, zeta, scaling);
			}
		}
		else if(0 == get_accel_type())
		{
			accel_lift_op4s_main_s(arr+off, (to_even(len-off)-4)/2, alpha, beta, gamma, delta, zeta, scaling);
		}
		else if(2 == get_accel_type())
		{
			// empty
		}
		else if(3 == get_accel_type())
		{
			off = 0;
			accel_lift_op4s_main_pb_s(arr+off, (to_even(len-off)-4)/2, alpha, beta, gamma, delta, zeta, scaling);
		}
		else if(4 == get_accel_type())
		{
			accel_lift_op4s_main_dl_s(arr+off, (to_even(len-off)-4)/2, alpha, beta, gamma, delta, zeta, scaling);
		}
		else if(5 == get_accel_type())
		{
			const int steps = (to_even(len-off)-4)/2;

			if( steps < 3 )
				accel_lift_op4s_main_s(arr+off, steps, alpha, beta, gamma, delta, zeta, scaling);
			else
				accel_lift_op4s_main_sdl_ref_s(arr+off, steps, alpha, beta, gamma, delta, zeta, scaling);
		}
		else if(6 == get_accel_type())
		{
			const int steps = (to_even(len-off)-4)/2;

			if( steps < 3 )
				accel_lift_op4s_main_s(arr+off, steps, alpha, beta, gamma, delta, zeta, scaling);
			else
				accel_lift_op4s_main_sdl2_ref_s(arr+off, steps, alpha, beta, gamma, delta, zeta, scaling);
		}
		else if(7 == get_accel_type())
		{
			const int steps = (to_even(len-off)-4)/2;

			if( steps < 3 )
				accel_lift_op4s_main_s(arr+off, steps, alpha, beta, gamma, delta, zeta, scaling);
			else
				accel_lift_op4s_main_sdl6_ref_s(arr+off, steps, alpha, beta, gamma, delta, zeta, scaling);
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
	if(N < 2)
	{
		if(1 == N)
			dst_l[0] = src[0] * dwt_cdf97_s1_s;
		return;
	}

	// copy src into tmp
	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		float *tmp_local = calc_temp_offset_s(tmp, w);
		const float *src_local = calc_data_offset_const_s(src, w);

		if( is_valid_data_step_s( src_local ) )
		{
			dwt_util_memcpy_stride_s(tmp_local, sizeof(float), src_local, stride, N);
		}
	}

	accel_lift_op4s_s(tmp, 1, N, -dwt_cdf97_p1_s, dwt_cdf97_u1_s, -dwt_cdf97_p2_s, dwt_cdf97_u2_s, dwt_cdf97_s1_s, +1);

	// copy tmp into dst
	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		float *tmp_local = calc_temp_offset_s(tmp, w);
		float *dst_l_local = calc_data_offset_s(dst_l, w);
		float *dst_h_local = calc_data_offset_s(dst_h, w);

		if( is_valid_data_step_s( dst_l_local ) )
		{
			dwt_util_memcpy_stride_s(dst_l_local, stride, tmp_local+0, 2*sizeof(float),  ceil_div2(N));
			dwt_util_memcpy_stride_s(dst_h_local, stride, tmp_local+1, 2*sizeof(float), floor_div2(N));
		}
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
	if(N < 2)
	{
		if(1 == N)
			dst[0] = src_l[0] * dwt_cdf97_s2_s; // FIXME: 1/zeta
		return;
	}

	// copy src into tmp
	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		float *tmp_local = calc_temp_offset_s(tmp, w);
		const float *src_l_local = calc_data_offset_const_s(src_l, w);
		const float *src_h_local = calc_data_offset_const_s(src_h, w);

		if( is_valid_data_step_s(src_l_local) )
		{
			dwt_util_memcpy_stride_s(tmp_local+0, 2*sizeof(float), src_l_local, stride,  ceil_div2(N));
			dwt_util_memcpy_stride_s(tmp_local+1, 2*sizeof(float), src_h_local, stride, floor_div2(N));
		}
	}

	accel_lift_op4s_s(tmp, 0, N, -dwt_cdf97_u2_s, dwt_cdf97_p2_s, -dwt_cdf97_u1_s, dwt_cdf97_p1_s, dwt_cdf97_s1_s, -1);

	// copy tmp into dst
	for(int w = 0; w < dwt_util_get_num_workers(); w++)
	{
		float *tmp_local = calc_temp_offset_s(tmp, w);
		float *dst_local = calc_data_offset_s(dst, w);

		if( is_valid_data_step_s(dst_local) )
		{
			dwt_util_memcpy_stride_s(dst_local, stride, tmp_local, sizeof(float), N);
		}
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
	if( op == dwt_util_global_active_op )
		return;
	//WAL_CHECK( wal_mb2pb(worker, 0) );

	//WAL_CHECK( wal_bce_jk_sync_operation(worker) );

	for(int w = 0; w < get_total_workers(); w++)
	{
		WAL_CHECK( wal_reset_worker(worker[w]) );
	}

	switch(op)
	{
		case DWT_OP_LIFT4SA:
		{
			for(int w = 0; w < get_total_workers(); w++)
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

			for(int w = 0; w < get_total_workers(); w++)
			{
				WAL_CHECK( wal_dma_configure(worker[w], 0, addr, 0, WAL_BCE_JSY_DMEM_B, 0, size) );
				WAL_CHECK( wal_dma_start(worker[w], 0, WAL_DMA_REQ_RD) );
				while( wal_dma_isbusy(worker[w], 0x01) )
					;
			}

			free(addr);
		}
		break;
		case DWT_OP_LIFT4SB:
		{
			for(int w = 0; w < get_total_workers(); w++)
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

			for(int w = 0; w < get_total_workers(); w++)
			{
				WAL_CHECK( wal_dma_configure(worker[w], 0, addr, 0, WAL_BCE_JSY_DMEM_B, 0, size) );
				WAL_CHECK( wal_dma_start(worker[w], 0, WAL_DMA_REQ_RD) );
				while( wal_dma_isbusy(worker[w], 0x01) )
					;
			}

			free(addr);
		}
		break;
		default:
			dwt_util_abort();
	}

	dwt_util_global_active_op = op;
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

#ifdef microblaze
	dwt_util_switch_op(DWT_OP_LIFT4SA);
#endif
	const int size_o_big_min = min(size_o_big_x,size_o_big_y);
	const int size_o_big_max = max(size_o_big_x,size_o_big_y);

	// FIXME: allocate temp[max_threads][temp_size] and remove private() in omp parallel
	// FIXME: OpenMP cannot use this in private()
#ifdef microblaze
	#define TEMP_OFFSET 1
	float *temp = dwt_util_allocate_vec_s(calc_and_set_temp_size(size_o_big_max));
#else
	//#define TEMP_OFFSET 0
	// HACK FIXME: __attribute__ ((aligned (16)))
	#define TEMP_OFFSET 3
	float temp[calc_and_set_temp_size(size_o_big_max)] __attribute__ ((aligned (16)));
	
	if( !is_aligned_16(temp) )
		dwt_util_abort();
#endif
	if(NULL == temp)
		abort(); // FIXME

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

#ifdef _OPENMP
		const int threads_segment_y = ceil_div(size_o_src_y, dwt_util_get_num_threads());
		const int threads_segment_x = ceil_div(size_o_src_x, dwt_util_get_num_threads());
#endif
		const int workers_segment_y = ceil_div(size_o_src_y, dwt_util_get_num_workers());
		const int workers_segment_x = ceil_div(size_o_src_x, dwt_util_get_num_workers());

#ifdef microblaze
		set_data_step_s( (intptr_t)addr2_s(ptr,workers_segment_y,0,stride_x,stride_y) - (intptr_t)addr2_s(ptr,0,0,stride_x,stride_y) );
		set_data_limit_s( addr2_s(ptr,size_o_src_y,0,stride_x,stride_y) );
#endif
		#pragma omp parallel for private(temp) schedule(static, threads_segment_y)
		for(int y = 0; y < workers_segment_y; y++)
			dwt_cdf97_f_ex_stride_s(
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,0,stride_x,stride_y),
				addr2_s(ptr,y,size_o_dst_x,stride_x,stride_y),
				temp + TEMP_OFFSET, // HACK: +1, FIXME: can this work under OpenMP?
				size_i_src_x,
				stride_y);
#ifdef microblaze
		set_data_step_s( (intptr_t)addr2_s(ptr,0,workers_segment_x,stride_x,stride_y) - (intptr_t)addr2_s(ptr,0,0,stride_x,stride_y) );
		set_data_limit_s( addr2_s(ptr,0,size_o_src_x,stride_x,stride_y) );
#endif
		#pragma omp parallel for private(temp) schedule(static, threads_segment_x)
		for(int x = 0; x < workers_segment_x; x++)
			dwt_cdf97_f_ex_stride_s(
				addr2_s(ptr,0,x,stride_x,stride_y),
				addr2_s(ptr,0,x,stride_x,stride_y),
				addr2_s(ptr,size_o_dst_y,x,stride_x,stride_y),
				temp + TEMP_OFFSET, // HACK: +1, FIXME: can this work under OpenMP?
				size_i_src_y,
				stride_x);

		if(zero_padding)
		{
			#pragma omp parallel for schedule(static, threads_segment_y)
			for(int y = 0; y < size_o_src_y; y++)
				dwt_zero_padding_f_stride_s(
					addr2_s(ptr,y,0,stride_x,stride_y),
					addr2_s(ptr,y,size_o_dst_x,stride_x,stride_y),
					size_i_src_x,
					size_o_dst_x,
					size_o_src_x-size_o_dst_x,
					stride_y);
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

#undef TEMP_OFFSET
#ifdef microblaze
	free(temp);
#endif

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

#ifdef microblaze
	dwt_util_switch_op(DWT_OP_LIFT4SB);
#endif
	const int size_o_big_min = min(size_o_big_x,size_o_big_y);
	const int size_o_big_max = max(size_o_big_x,size_o_big_y);

	// FIXME: allocate temp[max_threads][temp_size] and remove private() in omp parallel
	// FIXME: OpenMP cannot use this in private()
#ifdef microblaze
	float *temp = dwt_util_allocate_vec_s(calc_and_set_temp_size(size_o_big_max));
#else
	float temp[calc_and_set_temp_size(size_o_big_max)];
#endif
	if(NULL == temp)
		abort(); // FIXME

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

#ifdef _OPENMP
		const int threads_segment_y = ceil_div(size_o_dst_y, dwt_util_get_num_threads());
		const int threads_segment_x = ceil_div(size_o_dst_x, dwt_util_get_num_threads());
#endif
		const int workers_segment_y = ceil_div(size_o_dst_y, dwt_util_get_num_workers());
		const int workers_segment_x = ceil_div(size_o_dst_x, dwt_util_get_num_workers());

#ifdef microblaze
		set_data_step_s( (intptr_t)addr2_s(ptr,workers_segment_y,0,stride_x,stride_y) - (intptr_t)addr2_s(ptr,0,0,stride_x,stride_y) );
		set_data_limit_s( addr2_s(ptr,size_o_dst_y,0,stride_x,stride_y) );
#endif
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
		set_data_step_s( (intptr_t)addr2_s(ptr,0,workers_segment_x,stride_x,stride_y) - (intptr_t)addr2_s(ptr,0,0,stride_x,stride_y) );
		set_data_limit_s( addr2_s(ptr,0,size_o_dst_x,stride_x,stride_y) );
#endif
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
			#pragma omp parallel for schedule(static, threads_segment_y)
			for(int y = 0; y < size_o_dst_y; y++)
				dwt_zero_padding_i_stride_s(
					addr2_s(ptr,y,0,stride_x,stride_y),
					size_i_dst_x,
					size_o_dst_x,
					stride_y);
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

#ifdef microblaze
	free(temp);
#endif

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
	return get_total_workers();
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
	set_active_workers(num_workers);
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
	return get_active_workers();
#else
	return 1;
#endif
}

void dwt_util_init()
{
	FUNC_BEGIN;

#ifdef microblaze
	for(int w = 0; w < get_total_workers(); w++)
	{
		WAL_CHECK( wal_init_worker(worker[w]) );

		// FIXME: translate DWT_OP_LIFT4SA into WAL_PBID_P0 by function
	
		WAL_CHECK( wal_set_firmware(worker[w], WAL_PBID_P0 /*DWT_OP_LIFT4SA*/, fw_fp01_lift4sa, -1) );

		WAL_CHECK( wal_set_firmware(worker[w], WAL_PBID_P1 /*DWT_OP_LIFT4SB*/, fw_fp01_lift4sb, -1) );

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
	for(int w = 0; w < get_total_workers(); w++)
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
	for(int w = 0; w < get_total_workers(); w++)
	{
		// FIXME: is this legal? although the operation was not running?
		wal_end_operation(worker[w]);

		// deinitialize worker
		wal_done_worker(worker[w]);
	}
#endif /* microblaze */

	abort();

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

	int err = 0;

	for(int y = 0; y < size_i_big_y; y++)
	{
		for(int x = 0; x < size_i_big_x; x++)
		{
			const float px = *addr2_s(ptr, y, x, stride_x, stride_y);

			int val = (target_max_value*px/max_value);

			if( px - 1e-3f > max_value )
			{
				if( !err++)
					dwt_util_log(LOG_WARN, "%s: Maximum pixel intensity exceeded (%f > %f). Such an incident will be reported only once.\n", __FUNCTION__, px, max_value);
			}

			if( px > max_value )
			{
				val = target_max_value;
			}

			if( px + 1e-3f < 0.0f )
			{
				if( !err++ )
					dwt_util_log(LOG_WARN, "%s: Minimum pixel intensity exceeded (%f < %f). Such an incident will be reported only once.\n", __FUNCTION__, px, 0.0f);
			}

			if( px < 0.0f )
			{
				val = 0;
			}

			if( fprintf(file, "%i\n", val) < 0)
			{
				dwt_util_log(LOG_WARN, "%s: error writing into file.\n", __FUNCTION__);
				fclose(file);
				return 1;
			}
		}
	}

	fclose(file);

	if( err )
		dwt_util_log(LOG_WARN, "%s: %i errors ocurred while saving a file.\n", __FUNCTION__, err);

	return 0;
}

void dwt_util_set_accel(
	int accel_type)
{
	set_accel_type(accel_type);
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

// 4-bytes alignment
float *dwt_util_allocate_4_vec_s(int size)
{
	assert( is_even(size) );

	float *addr = (float *)0;

	// http://git.uclibc.org/uClibc/tree/include - memalign i posix_memalign
	addr = (float *)memalign(4, sizeof(float) * size);

	assert( is_aligned_4(addr) );

	return addr;
}

// 8-bytes alignment
float *dwt_util_allocate_8_vec_s(int size)
{
	assert( is_even(size) );

	float *addr = (float *)0;

	// http://git.uclibc.org/uClibc/tree/include - memalign, posix_memalign
	addr = (float *)memalign(8, sizeof(float) * size);

	assert( is_aligned_8(addr) );

	return addr;
}

// 16-bytes alignment
float *dwt_util_allocate_16_vec_s(int size)
{
	assert( is_even(size) );

	float *addr = (float *)0;

	// http://git.uclibc.org/uClibc/tree/include - memalign, posix_memalign
	addr = (float *)memalign(16, sizeof(float) * size);

	assert( is_aligned_16(addr) );

	return addr;
}

float *dwt_util_allocate_vec_s(int size)
{
	assert( is_even(size) );

	float *addr = (float *)0;

	// http://git.uclibc.org/uClibc/tree/include - memalign, posix_memalign
	addr = (float *)memalign(8, sizeof(float) * size);

	assert( is_aligned_8(addr) );

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
		dwt_util_log(LOG_TEST, "allocate vector of %i floats...\n", i);
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
	for(int w = 0; w < get_total_workers(); w++)
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

		flush_cache_s(addr, size);

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

	flockfile(stream);

	ret += dwt_util_fprintf(stream, prefix[level]);

	va_list ap;

	va_start(ap, format);
	ret += dwt_util_vfprintf(stream, format, ap);
	va_end(ap);

	fflush(stream);

	funlockfile(stream);

	return ret;
}

static
const char *node()
{
	FILE *f = fopen("/proc/sys/kernel/hostname", "r");
	if(f)
	{
		// FIXME: HOST_NAME_MAX, HOSTNAME_LENGTH, MAXHOSTNAMELEN
#define MAX_NODE_LEN 256
		static char buff[MAX_NODE_LEN]; // NOTE: global variable
		const char *ret = fgets(buff, MAX_NODE_LEN, f);
		fclose(f);

		if(ret)
		{
			char *nl = strchr(buff, '\n');
			if(nl)
				*nl = 0;

			return buff;
		}
		else
			return "unknown";
	}
	else
		return "unknown";
}

const char *dwt_util_node()
{
		return node();
}

static
const char *appname()
{
	FILE *f = fopen("/proc/self/cmdline", "r");
	if(f)
	{
		// FIXME: PAGE_SIZE
#define MAX_CMDLINE_LEN 4096
		static char buff[MAX_CMDLINE_LEN]; // NOTE: global variable
		const char *ret = fgets(buff, MAX_CMDLINE_LEN, f);
		fclose(f);
		
		if(ret)
			return basename(buff);
		else
			return "unknown";
	}
	else
		return "unknown";
}

const char *dwt_util_appname()
{
	return appname();
}

static
int find_dfa_seq(int N)
{
	int state = 1;

	int count = 0;

	do
	{
		const int addr = 2 * state;

		state = addr - N * (addr >= N);

		count++;

		if( 1 == state )
			return count;
	}
	while( count < 2*N );

	return 0;
}

/**
 * @brief Variant of Fermat primality test for base-2.
 */
static
int is_prime(int N)
{
	// 2 is prime
	if( 2 == N )
		return 1;

	// even numbers are not primes, i.e. 0, 2, 4, 6, 8, ...
	if( !(N & 1) )
		return 0;

	// negative numbers and unity are not prime numbers, i.e. ..., -2, -1, 0, 1
	if( N < 2 )
		return 0;

	// number of zeros after leading one-bit in left side of Fermat's little theorem with base 2
	const int d = N - 1;

	// length of zero-bit sequence after leading one-bit accepted by DFA which accepts numbers congruent to 1 modulo N
	const int c = find_dfa_seq(N);

	// can DFA accept a big number in the left side of Fermat's little theorem?
	const int r = d % c;

	// if can then we got probably prime
	if( 0 == r )
		return 1;

	return 0;
}

int dwt_util_is_prime(int N)
{
	return is_prime(N);
}

/**
 * @brief Returns smallest prime not less than N.
 */
static
int next_prime(int N)
{
	if( N <= 2 )
		return 2;

	N |= 1;

	while( !is_prime(N) )
		N += 2;

	return N;
}

static
int get_opt_stride(int min_stride)
{
	assert( min_stride > 0 );

#ifdef microblaze
	// align to 32 bits due to MicroBlaze constraints
	return align_8(min_stride);
#else
	// find prime number not lesser than min_stride
	return next_prime(min_stride);
#endif
}

int dwt_util_get_opt_stride(int min_stride)
{
	return get_opt_stride(min_stride);
}

void dwt_util_subband(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y,
	int size_i_big_x,
	int size_i_big_y,
	int j_max,
	enum subbands band,
	void **dst_ptr,
	int *dst_size_x,
	int *dst_size_y)
{
	assert( ptr != NULL && size_i_big_x >= 0 && size_i_big_y >= 0 && size_o_big_x >= 0 && size_o_big_y >= 0 );

	int inner_H_x = 0;
	int inner_H_y = 0;
	int inner_L_x = size_i_big_x;
	int inner_L_y = size_i_big_y;
	int outer_x = size_o_big_x;
	int outer_y = size_o_big_y;

	for(int j = 1; j <= j_max; j++)
	{
		inner_H_x = floor_div2(inner_L_x);
		inner_H_y = floor_div2(inner_L_y);
		inner_L_x = ceil_div2 (inner_L_x);
		inner_L_y = ceil_div2 (inner_L_y);
		outer_x   = ceil_div2 (outer_x);
		outer_y   = ceil_div2 (outer_y);
	}

	switch(band)
	{
		case DWT_LL:
			*dst_ptr = addr2(ptr,
				0, 0,
				stride_x, stride_y);
			*dst_size_x = inner_L_x;
			*dst_size_y = inner_L_y;
			break;
		case DWT_HL:
			*dst_ptr = addr2(ptr,
				0, outer_x,
				stride_x, stride_y);
			*dst_size_x = inner_H_x;
			*dst_size_y = inner_L_y;
			break;
		case DWT_LH:
			*dst_ptr = addr2(ptr,
				outer_y, 0,
				stride_x, stride_y);
			*dst_size_x = inner_L_x;
			*dst_size_y = inner_H_y;
			break;
		case DWT_HH:
			*dst_ptr = addr2(ptr,
				outer_y, outer_x,
				stride_x, stride_y);
			*dst_size_x = inner_H_x;
			*dst_size_y = inner_H_y;
			break;
	}
}

void dwt_util_subband_s(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y,
	int size_i_big_x,
	int size_i_big_y,
	int j_max,
	enum subbands band,
	void **dst_ptr,
	int *dst_size_x,
	int *dst_size_y)
{
	dwt_util_subband(
		ptr,
		stride_x,
		stride_y,
		size_o_big_x,
		size_o_big_y,
		size_i_big_x,
		size_i_big_y,
		j_max,
		band,
		dst_ptr,
		dst_size_x,
		dst_size_y);
}

void dwt_util_subband_d(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y,
	int size_i_big_x,
	int size_i_big_y,
	int j_max,
	enum subbands band,
	void **dst_ptr,
	int *dst_size_x,
	int *dst_size_y)
{
	dwt_util_subband(
		ptr,
		stride_x,
		stride_y,
		size_o_big_x,
		size_o_big_y,
		size_i_big_x,
		size_i_big_y,
		j_max,
		band,
		dst_ptr,
		dst_size_x,
		dst_size_y);
}
