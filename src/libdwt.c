/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Fast wavelet transform implemented via lifting scheme.
 */
#include "libdwt.h"

#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <limits.h>

#ifdef _OPENMP
#include <omp.h>
#endif

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
static const double dwt_cdf97_s2_d =  1/1.1496043988602;

static const float dwt_cdf97_p1_s =    1.58613434342059;
static const float dwt_cdf97_u1_s =   -0.0529801185729;
static const float dwt_cdf97_p2_s =   -0.8829110755309;
static const float dwt_cdf97_u2_s =    0.4435068520439;
static const float dwt_cdf97_s1_s =    1.1496043988602;
static const float dwt_cdf97_s2_s =  1/1.1496043988602;
/**@}*/

/**
 * @brief Shift series
 */
static
int or_shift(int shift, int x)
{
	x = shift>1 ? or_shift(shift>>1, x) : x;
	return x | x >> shift;
}

/**
 * @brief Power of two using greater or equal to x, i.e. 2^(ceil(log_2(x)).
 */
static
int pot(int x)
{
	assert(x > 0);

	return or_shift(sizeof(int) * CHAR_BIT / 2, x - 1) + 1;
}

/**
 * @brief Number of 1-bits in x, in parallel.
 */
static
int bits(int x)
{
	assert(sizeof(int) <= 4);

	#define BREP(byte) ( (byte) | (byte) << 8 | (byte) << 16 | (byte) << 24 )
	x -= x >> 1 & BREP(0x55);
	x = (x & BREP(0x33)) + (x >> 2 & BREP(0x33));
	x = (x + (x >> 4)) & BREP(0x0f);
	return x * BREP(0x01) >> (sizeof(int) * CHAR_BIT - CHAR_BIT);
	#undef BREP
}

/**
 * @brief Smallest integer not less than the base 2 logarithm of x, i.e. ceil(log_2(x)).
 * @returns (int)ceil(log2(x))
 */
static
int ceil_log2(int x)
{
	return bits(pot(x) - 1);
}

/**
 * @returns (int)ceil(x/(double)y)
 */
static
int ceil_div(int x, int y)
{
	return (x + y - 1) / y;
}

/**
 * @returns (int)ceil(i/(double)(1<<j))
 */
static
int ceil_div_pow2(int i, int j)
{
	return (i + (1 << j) - 1) >> j;
}

/**
 * @brief Minimum of two integers.
 */
static
int min(int a, int b)
{
	return a>b ? b : a;
}

/**
 * @brief Maximum of two integers.
 */
static
int max(int a, int b)
{
	return a>b ? a : b;
}

/**
 * @brief Helper function returning address of given element.
 *
 * Evaluate address of (i) image element, returns (const double *).
 */
static
const double *addr1_const_d(const void *ptr, int i, int stride)
{
	return (const double *)((const char *)ptr+i*stride);
}

/**
 * @brief Helper function returning address of given element.
 *
 * Evaluate address of (i) image element, returns (const double *).
 */
static
const float *addr1_const_s(const void *ptr, int i, int stride)
{
	return (const float *)((const char *)ptr+i*stride);
}

/**
 * @brief Helper function returning address of given element.
 *
 * Evaluate address of (i) image element, returns (double *).
 */
static
double *addr1_d(void *ptr, int i, int stride)
{
	return (double *)((char *)ptr+i*stride);
}

/**
 * @brief Helper function returning address of given element.
 *
 * Evaluate address of (i) image element, returns (double *).
 */
static
float *addr1_s(void *ptr, int i, int stride)
{
	return (float *)((char *)ptr+i*stride);
}

/**
 * @brief Helper function returning address of given pixel.
 *
 * Evaluate address of (x,y) image element, returns (double *).
 */
static
double *addr2_d(void *ptr, int y, int x, int stride_x, int stride_y)
{
	return (double *)((char *)ptr+y*stride_x+x*stride_y);
}

/**
 * @brief Helper function returning address of given pixel.
 *
 * Evaluate address of (x,y) image element, returns (double *).
 */
static
float *addr2_s(void *ptr, int y, int x, int stride_x, int stride_y)
{
	return (float *)((char *)ptr+y*stride_x+x*stride_y);
}

void dwt_cdf97_f(
	const double *src,
	double *dst,
	double *tmp,
	int N
)
{
	dwt_cdf97_f_ex(
		src,
		dst,
		dst+N/2+(N&1),
		tmp,
		N
	);
}

void dwt_cdf97_f_d(
	const double *src,
	double *dst,
	double *tmp,
	int N
)
{
	dwt_cdf97_f_ex(
		src,
		dst,
		dst+N/2+(N&1),
		tmp,
		N
	);
}

void dwt_cdf97_f_s(
	const float *src,
	float *dst,
	float *tmp,
	int N
)
{
	dwt_cdf97_f_ex_s(
		src,
		dst,
		dst+N/2+(N&1),
		tmp,
		N
	);
}

void dwt_cdf97_i(
	const double *src,
	double *dst,
	double *tmp,
	int N
)
{
	dwt_cdf97_i_ex(
		src,
		src+N/2+(N&1),
		dst,
		tmp,
		N
	);
}

void dwt_cdf97_i_d(
	const double *src,
	double *dst,
	double *tmp,
	int N
)
{
	dwt_cdf97_i_ex(
		src,
		src+N/2+(N&1),
		dst,
		tmp,
		N
	);
}

void dwt_cdf97_i_s(
	const float *src,
	float *dst,
	float *tmp,
	int N
)
{
	dwt_cdf97_i_ex_s(
		src,
		src+N/2+(N&1),
		dst,
		tmp,
		N
	);
}

void dwt_cdf97_f_ex(
	const double *src,
	double *dst_l,
	double *dst_h,
	double *tmp,
	int N
)
{
	dwt_cdf97_f_ex_stride(
		src,
		dst_l,
		dst_h,
		tmp,
		N,
		sizeof(double)
	);
}

void dwt_cdf97_f_ex_d(
	const double *src,
	double *dst_l,
	double *dst_h,
	double *tmp,
	int N
)
{
	dwt_cdf97_f_ex_stride(
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
	int N
)
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

void dwt_cdf97_f_ex_stride(
	const double *src,
	double *dst_l,
	double *dst_h,
	double *tmp,
	int N,
	int stride
)
{
	dwt_cdf97_f_ex_stride_d(
		src,
		dst_l,
		dst_h,
		tmp,
		N,
		stride
	);
}

void dwt_cdf97_f_ex_stride_d(
	const double *src,
	double *dst_l,
	double *dst_h,
	double *tmp,
	int N,
	int stride
)
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
	for(int i=0; i<N; i++)
		tmp[i] = *addr1_const_d(src,i,stride);

	// predict 1 + update 1
	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] -= dwt_cdf97_p1_d * (tmp[i-1] + tmp[i+1]);

	if(N&1)
		tmp[N-1] += 2 * dwt_cdf97_u1_d * tmp[N-2];
	else
		tmp[N-1] -= 2 * dwt_cdf97_p1_d * tmp[N-2];;

	tmp[0] += 2 * dwt_cdf97_u1_d * tmp[1];

	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] += dwt_cdf97_u1_d * (tmp[i-1] + tmp[i+1]);

	// predict 2 + update 2
	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] -= dwt_cdf97_p2_d * (tmp[i-1] + tmp[i+1]);

	if(N&1)
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
	for(int i=0; i<N/2+(N&1); i++)
		*addr1_d(dst_l,i,stride) = tmp[2*i  ];
	for(int i=0; i<N/2      ; i++)
		*addr1_d(dst_h,i,stride) = tmp[2*i+1];
}

void dwt_cdf97_f_ex_stride_s(
	const float *src,
	float *dst_l,
	float *dst_h,
	float *tmp,
	int N,
	int stride
)
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
	for(int i=0; i<N; i++)
		tmp[i] = *addr1_const_s(src,i,stride);

	// predict 1 + update 1
	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] -= dwt_cdf97_p1_s * (tmp[i-1] + tmp[i+1]);

	if(N&1)
		tmp[N-1] += 2 * dwt_cdf97_u1_s * tmp[N-2];
	else
		tmp[N-1] -= 2 * dwt_cdf97_p1_s * tmp[N-2];;

	tmp[0] += 2 * dwt_cdf97_u1_s * tmp[1];

	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] += dwt_cdf97_u1_s * (tmp[i-1] + tmp[i+1]);

	// predict 2 + update 2
	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] -= dwt_cdf97_p2_s * (tmp[i-1] + tmp[i+1]);

	if(N&1)
		tmp[N-1] += 2 * dwt_cdf97_u2_s * tmp[N-2];
	else
		tmp[N-1] -= 2 * dwt_cdf97_p2_s * tmp[N-2];

	tmp[0] += 2 * dwt_cdf97_u2_s * tmp[1];

	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] += dwt_cdf97_u2_s * (tmp[i-1] + tmp[i+1]);

	// scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s1_s;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s2_s;

	// copy tmp into dst
	for(int i=0; i<N/2+(N&1); i++)
		*addr1_s(dst_l,i,stride) = tmp[2*i  ];
	for(int i=0; i<N/2      ; i++)
		*addr1_s(dst_h,i,stride) = tmp[2*i+1];
}


void dwt_cdf97_i_ex(
	const double *src_l,
	const double *src_h,
	double *dst,
	double *tmp,
	int N
)
{
	dwt_cdf97_i_ex_stride(
		src_l,
		src_h,
		dst,
		tmp,
		N,
		sizeof(double)
	);
}

void dwt_cdf97_i_ex_d(
	const double *src_l,
	const double *src_h,
	double *dst,
	double *tmp,
	int N
)
{
	dwt_cdf97_i_ex_stride(
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
	int N
)
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

void dwt_cdf97_i_ex_stride(
	const double *src_l,
	const double *src_h,
	double *dst,
	double *tmp,
	int N,
	int stride
)
{
	dwt_cdf97_i_ex_stride_d(
		src_l,
		src_h,
		dst,
		tmp,
		N,
		stride
	);
}

void dwt_cdf97_i_ex_stride_d(
	const double *src_l,
	const double *src_h,
	double *dst,
	double *tmp,
	int N,
	int stride
)
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
	for(int i=0; i<N/2+(N&1); i++)
		tmp[2*i  ] = *addr1_const_d(src_l,i,stride);
	for(int i=0; i<N/2      ; i++)
		tmp[2*i+1] = *addr1_const_d(src_h,i,stride);

	// inverse scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s2_d;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s1_d;

	// backward update 2 + backward predict 2
	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] -= dwt_cdf97_u2_d * (tmp[i-1] + tmp[i+1]);

	tmp[0] -= 2 * dwt_cdf97_u2_d * tmp[1];

	if(N&1)
		tmp[N-1] -= 2 * dwt_cdf97_u2_d * tmp[N-2];
	else
		tmp[N-1] += 2 * dwt_cdf97_p2_d * tmp[N-2];

	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] += dwt_cdf97_p2_d * (tmp[i-1] + tmp[i+1]);

	// backward update 1 + backward predict 1
	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] -= dwt_cdf97_u1_d * (tmp[i-1] + tmp[i+1]);

	tmp[0] -= 2 * dwt_cdf97_u1_d * tmp[1];

	if(N&1)
		tmp[N-1] -= 2 * dwt_cdf97_u1_d * tmp[N-2];
	else
		tmp[N-1] += 2 * dwt_cdf97_p1_d * tmp[N-2];

	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] += dwt_cdf97_p1_d * (tmp[i-1] + tmp[i+1]);

	// copy tmp into dst
	for(int i=0; i<N; i++)
		*addr1_d(dst,i,stride) = tmp[i];
}

void dwt_cdf97_i_ex_stride_s(
	const float *src_l,
	const float *src_h,
	float *dst,
	float *tmp,
	int N,
	int stride
)
{
	assert(!( N < 0 || NULL == src_l || NULL == src_h || NULL == dst || NULL == tmp || 0 == stride ));

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst[0] = src_l[0] * dwt_cdf97_s2_s;
		return;
	}

	// copy src into tmp
	for(int i=0; i<N/2+(N&1); i++)
		tmp[2*i  ] = *addr1_const_s(src_l,i,stride);
	for(int i=0; i<N/2      ; i++)
		tmp[2*i+1] = *addr1_const_s(src_h,i,stride);

	// inverse scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s2_s;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s1_s;

	// backward update 2 + backward predict 2
	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] -= dwt_cdf97_u2_s * (tmp[i-1] + tmp[i+1]);

	tmp[0] -= 2 * dwt_cdf97_u2_s * tmp[1];

	if(N&1)
		tmp[N-1] -= 2 * dwt_cdf97_u2_s * tmp[N-2];
	else
		tmp[N-1] += 2 * dwt_cdf97_p2_s * tmp[N-2];

	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] += dwt_cdf97_p2_s * (tmp[i-1] + tmp[i+1]);

	// backward update 1 + backward predict 1
	for(int i=2; i<N-(N&1); i+=2)
		tmp[i] -= dwt_cdf97_u1_s * (tmp[i-1] + tmp[i+1]);

	tmp[0] -= 2 * dwt_cdf97_u1_s * tmp[1];

	if(N&1)
		tmp[N-1] -= 2 * dwt_cdf97_u1_s * tmp[N-2];
	else
		tmp[N-1] += 2 * dwt_cdf97_p1_s * tmp[N-2];

	for(int i=1; i<N-2+(N&1); i+=2)
		tmp[i] += dwt_cdf97_p1_s * (tmp[i-1] + tmp[i+1]);

	// copy tmp into dst
	for(int i=0; i<N; i++)
		*addr1_s(dst,i,stride) = tmp[i];
}

void dwt_zero_padding_f(
	double *dst_l,
	double *dst_h,
	int N,
	int N_dst_L,
	int N_dst_H
)
{
	dwt_zero_padding_f_stride(
		dst_l,
		dst_h,
		N,
		N_dst_L,
		N_dst_H,
		sizeof(double)
	);
}

void dwt_zero_padding_f_d(
	double *dst_l,
	double *dst_h,
	int N,
	int N_dst_L,
	int N_dst_H
)
{
	dwt_zero_padding_f_stride(
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
	int N_dst_H
)
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

void dwt_zero_padding_f_stride(
	double *dst_l,
	double *dst_h,
	int N,
	int N_dst_L,
	int N_dst_H,
	int stride
)
{
	dwt_zero_padding_f_stride_d(
		dst_l,
		dst_h,
		N,
		N_dst_L,
		N_dst_H,
		stride
	);
}

void dwt_zero_padding_f_stride_d(
	double *dst_l,
	double *dst_h,
	int N,
	int N_dst_L,
	int N_dst_H,
	int stride
)
{
	assert(!( N < 0 || N_dst_L < 0 || N_dst_H < 0 || 0 != ((N_dst_L-N_dst_H)&~1) || NULL == dst_l || NULL == dst_h || 0 == stride ));

	if(N_dst_L || N_dst_H)
	{
		for(int i=N/2+(N&1); i<N_dst_L; i++)
			*addr1_d(dst_l,i,stride) = *addr1_d(dst_h,i,stride) = 0;

		if((N&1) && N/2<N_dst_H)
			*addr1_d(dst_h,N/2,stride) = 0;
	}
}

void dwt_zero_padding_f_stride_s(
	float *dst_l,
	float *dst_h,
	int N,
	int N_dst_L,
	int N_dst_H,
	int stride
)
{
	assert(!( N < 0 || N_dst_L < 0 || N_dst_H < 0 || 0 != ((N_dst_L-N_dst_H)&~1) || NULL == dst_l || NULL == dst_h || 0 == stride ));

	if(N_dst_L || N_dst_H)
	{
		for(int i=N/2+(N&1); i<N_dst_L; i++)
			*addr1_s(dst_l,i,stride) = *addr1_s(dst_h,i,stride) = 0;

		if((N&1) && N/2<N_dst_H)
			*addr1_s(dst_h,N/2,stride) = 0;
	}
}

void dwt_zero_padding_i(
	double *dst_l,
	int N,
	int N_dst
)
{
	dwt_zero_padding_i_d(
		dst_l,
		N,
		N_dst
	);
}

void dwt_zero_padding_i_d(
	double *dst_l,
	int N,
	int N_dst
)
{
	dwt_zero_padding_i_stride(
		dst_l,
		N,
		N_dst,
		sizeof(double)
	);
}

void dwt_zero_padding_i_s(
	float *dst_l,
	int N,
	int N_dst
)
{
	dwt_zero_padding_i_stride_s(
		dst_l,
		N,
		N_dst,
		sizeof(float)
	);
}

void dwt_zero_padding_i_stride(
	double *dst_l,
	int N,
	int N_dst,
	int stride
)
{
	dwt_zero_padding_i_stride_d(
		dst_l,
		N,
		N_dst,
		stride
	);
}

void dwt_zero_padding_i_stride_d(
	double *dst_l,
	int N,
	int N_dst,
	int stride
)
{
	assert(!( N < 0 || N_dst < 0 || NULL == dst_l || 0 == stride ));

	for(int i=N; i<N_dst; i++)
		*addr1_d(dst_l,i,stride) = 0;
}

void dwt_zero_padding_i_stride_s(
	float *dst_l,
	int N,
	int N_dst,
	int stride
)
{
	assert(!( N < 0 || N_dst < 0 || NULL == dst_l || 0 == stride ));

	for(int i=N; i<N_dst; i++)
		*addr1_s(dst_l,i,stride) = 0;
}

void dwt_cdf97_2f(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y,
	int size_i_big_x,
	int size_i_big_y,
	int *j_max_ptr,
	int decompose_one,
	int zero_padding
)
{
	dwt_cdf97_2f_d(
		ptr,
		stride_x,
		stride_y,
		size_o_big_x,
		size_o_big_y,
		size_i_big_x,
		size_i_big_y,
		j_max_ptr,
		decompose_one,
		zero_padding
	);
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
	int zero_padding
)
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
			dwt_cdf97_f_ex_stride(
				addr2_d(ptr,y,0,stride_x,stride_y),
				addr2_d(ptr,y,0,stride_x,stride_y),
				addr2_d(ptr,y,size_o_dst_x,stride_x,stride_y),
				temp,
				size_i_src_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_src_x; x++)
			dwt_cdf97_f_ex_stride(
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
				dwt_zero_padding_f_stride(
					addr2_d(ptr,y,0,stride_x,stride_y),
					addr2_d(ptr,y,size_o_dst_x,stride_x,stride_y),
					size_i_src_x,
					size_o_dst_x,
					size_o_src_x-size_o_dst_x,
					stride_y);
			#pragma omp parallel for schedule(static, ceil_div(size_o_src_x, omp_get_num_threads()))
			for(int x = 0; x < size_o_src_x; x++)
				dwt_zero_padding_f_stride(
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
	int zero_padding
)
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

void dwt_cdf97_2i(
	void *ptr,
	int stride_x,
	int stride_y,
	int size_o_big_x,
	int size_o_big_y,
	int size_i_big_x,
	int size_i_big_y,
	int j_max,
	int decompose_one,
	int zero_padding
)
{
	dwt_cdf97_2i_d(
		ptr,
		stride_x,
		stride_y,
		size_o_big_x,
		size_o_big_y,
		size_i_big_x,
		size_i_big_y,
		j_max,
		decompose_one,
		zero_padding
	);
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
	int zero_padding
)
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
			dwt_cdf97_i_ex_stride(
				addr2_d(ptr,y,0,stride_x,stride_y),
				addr2_d(ptr,y,size_o_src_x,stride_x,stride_y),
				addr2_d(ptr,y,0,stride_x,stride_y),
				temp,
				size_i_dst_x,
				stride_y);
		#pragma omp parallel for private(temp) schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
		for(int x = 0; x < size_o_dst_x; x++)
			dwt_cdf97_i_ex_stride(
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
				dwt_zero_padding_i_stride(
					addr2_d(ptr,y,0,stride_x,stride_y),
					size_i_dst_x,
					size_o_dst_x,
					stride_y);
			#pragma omp parallel for schedule(static, ceil_div(size_o_dst_x, omp_get_num_threads()))
			for(int x = 0; x < size_o_dst_x; x++)
				dwt_zero_padding_i_stride(
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
	int zero_padding
)
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
