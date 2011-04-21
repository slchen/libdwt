/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Fast wavelet transform implemented via lifting scheme.
 */
#include "libdwt.h"

#include <assert.h>
#include <stddef.h>
#include <stdlib.h>

/**
 * @{
 * These constants are found in S. Mallat. A Wavelet Tour of Signal Processing: The Sparse Way (Third Edition). 3rd edition, 2009 on page 370.
 */
static const double dwt_cdf97_p1 =    1.58613434342059;
static const double dwt_cdf97_u1 =   -0.0529801185729;
static const double dwt_cdf97_p2 =   -0.8829110755309;
static const double dwt_cdf97_u2 =    0.4435068520439;
static const double dwt_cdf97_s1  =   1.1496043988602;
static const double dwt_cdf97_s2  = 1/1.1496043988602;
/**@}*/

/**
 * @returns (int)ceil(log2(x))
 */
static
int ceil_log2(int x)
{
	assert(x > 0);

	x = (x-1) << 1;

	int y = 0;
	while(x > 1)
	{
		x >>= 1;
		y++;
	}
	return y;
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
 * Minimum of two integers.
 */
static
int min(int a, int b)
{
	return a>b ? b : a;
}

/**
 * Maximum of two integers.
 */
static
int max(int a, int b)
{
	return a>b ? a : b;
}

/**
 * Helper function returning address of given element.
 * Evaluate address of (i) image element, returns (const double *).
 */
static
const double *addr1_const(const void *ptr, int i, int stride)
{
	return (const double *)((const char *)ptr+i*stride);
}

/**
 * Helper function returning address of given element.
 * Evaluate address of (i) image element, returns (double *).
 */
static
double *addr1(void *ptr, int i, int stride)
{
	return (double *)((char *)ptr+i*stride);
}

/**
 * Helper function returning address of given pixel.
 * Evaluate address of (x,y) image element, returns (double *).
 */
static
double *addr2(void *ptr, int y, int x, int stride_x, int stride_y)
{
	return (double *)((char *)ptr+y*stride_x+x*stride_y);
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

void dwt_cdf97_f_ex_stride(
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
			dst_l[0] = src[0] * dwt_cdf97_s1;
		return;
	}

	// copy src into tmp
	for(int i=0; i<N; i++)
		tmp[i] = *addr1_const(src,i,stride);

	if(N&1)
	{
		// predict 1
		for(int i=1; i<N-1; i+=2)
			tmp[i] -= dwt_cdf97_p1 * (tmp[i-1] + tmp[i+1]);
	
		// update 1
		for(int i=2; i<N-1; i+=2)
			tmp[i] += dwt_cdf97_u1 * (tmp[i-1] + tmp[i+1]);
		tmp[0] += 2 * dwt_cdf97_u1 * tmp[1];
		tmp[N-1] += 2 * dwt_cdf97_u1 * tmp[N-2];
	
		// predict 2
		for(int i=1; i<N-1; i+=2)
			tmp[i] -= dwt_cdf97_p2 * (tmp[i-1] + tmp[i+1]);
	
		// update 2
		for(int i=2; i<N-1; i+=2)
			tmp[i] += dwt_cdf97_u2 * (tmp[i-1] + tmp[i+1]);
		tmp[0] += 2 * dwt_cdf97_u2 * tmp[1];
		tmp[N-1] += 2 * dwt_cdf97_u2 * tmp[N-2];
	}
	else
	{
		// predict 1
		for(int i=1; i<N-2; i+=2)
			tmp[i] -= dwt_cdf97_p1 * (tmp[i-1] + tmp[i+1]);
		tmp[N-1] -= 2 * dwt_cdf97_p1 * tmp[N-2];
	
		// update 1
		for(int i=2; i<N; i+=2)
			tmp[i] += dwt_cdf97_u1 * (tmp[i-1] + tmp[i+1]);
		tmp[0] += 2 * dwt_cdf97_u1 * tmp[1];
	
		// predict 2
		for(int i=1; i<N-2; i+=2)
			tmp[i] -= dwt_cdf97_p2 * (tmp[i-1] + tmp[i+1]);
		tmp[N-1] -= 2 * dwt_cdf97_p2 * tmp[N-2];
	
		// update 2
		for(int i=2; i<N; i+=2)
			tmp[i] += dwt_cdf97_u2 * (tmp[i-1] + tmp[i+1]);
		tmp[0] += 2 * dwt_cdf97_u2 * tmp[1];
	}

	// scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s1;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s2;

	// copy tmp into dst
	for(int i=0; i<N/2+(N&1); i++)
		*addr1(dst_l,i,stride) = tmp[2*i  ];
	for(int i=0; i<N/2      ; i++)
		*addr1(dst_h,i,stride) = tmp[2*i+1];
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

void dwt_cdf97_i_ex_stride(
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
			dst[0] = src_l[0] * dwt_cdf97_s2;
		return;
	}

	// copy src into tmp
	for(int i=0; i<N/2+(N&1); i++)
		tmp[2*i  ] = *addr1_const(src_l,i,stride);
	for(int i=0; i<N/2      ; i++)
		tmp[2*i+1] = *addr1_const(src_h,i,stride);

	// inverse scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s2;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s1;

	if(N&1)
	{
		// backward update 2
		for(int i=2; i<N-1; i+=2)
			tmp[i] -= dwt_cdf97_u2 * (tmp[i-1] + tmp[i+1]);
		tmp[0] -= 2 * dwt_cdf97_u2 * tmp[1];
		tmp[N-1] -= 2 * dwt_cdf97_u2 * tmp[N-2];
	
		// backward predict 2
		for(int i=1; i<N-1; i+=2)
			tmp[i] += dwt_cdf97_p2 * (tmp[i-1] + tmp[i+1]);
	
		// backward update 1
		for(int i=2; i<N-1; i+=2)
			tmp[i] -= dwt_cdf97_u1 * (tmp[i-1] + tmp[i+1]);
		tmp[0] -= 2 * dwt_cdf97_u1 * tmp[1];
		tmp[N-1] -= 2 * dwt_cdf97_u1 * tmp[N-2];
	
		// backward predict 1
		for(int i=1; i<N-1; i+=2)
			tmp[i] += dwt_cdf97_p1 * (tmp[i-1] + tmp[i+1]);
	}
	else
	{
		// backward update 2
		for(int i=2; i<N; i+=2)
			tmp[i] -= dwt_cdf97_u2 * (tmp[i-1] + tmp[i+1]);
		tmp[0] -= 2 * dwt_cdf97_u2 * tmp[1];
	
		// backward predict 2
		for(int i=1; i<N-2; i+=2)
			tmp[i] += dwt_cdf97_p2 * (tmp[i-1] + tmp[i+1]);
		tmp[N-1] += 2 * dwt_cdf97_p2 * tmp[N-2];
	
		// backward update 1
		for(int i=2; i<N; i+=2)
			tmp[i] -= dwt_cdf97_u1 * (tmp[i-1] + tmp[i+1]);
		tmp[0] -= 2 * dwt_cdf97_u1 * tmp[1];
	
		// backward predict 1
		for(int i=1; i<N-2; i+=2)
			tmp[i] += dwt_cdf97_p1 * (tmp[i-1] + tmp[i+1]);
		tmp[N-1] += 2 * dwt_cdf97_p1 * tmp[N-2];
	}

	// copy tmp into dst
	for(int i=0; i<N; i++)
		*addr1(dst,i,stride) = tmp[i];
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

void dwt_zero_padding_f_stride(
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
			*addr1(dst_l,i,stride) = *addr1(dst_h,i,stride) = 0;

		if((N&1) && N/2<N_dst_H)
			*addr1(dst_h,N/2,stride) = 0;
	}
}

void dwt_zero_padding_i(
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

void dwt_zero_padding_i_stride(
	double *dst_l,
	int N,
	int N_dst,
	int stride
)
{
	assert(!( N < 0 || N_dst < 0 || NULL == dst_l || 0 == stride ));

	for(int i=N; i<N_dst; i++)
		*addr1(dst_l,i,stride) = 0;
}

/*
 * notation:
 * size_{i|o}_{big|src|dst}_{x|y}
 * * i = inner, o = outer
 * * big = original, src = current source (j), dst = current destination (j+-1)
 * * x = horizontal, y = vertical
 * * e.g.: size_i_src_x = horizontal size of inner image viewed as source (at scale j) for FWT or inverse FWT
 */

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

		for(int y = 0; y < size_o_src_y; y++)
			dwt_cdf97_f_ex_stride(
				addr2(ptr,y,0,stride_x,stride_y),
				addr2(ptr,y,0,stride_x,stride_y),
				addr2(ptr,y,size_o_dst_x,stride_x,stride_y),
				temp,
				size_i_src_x,
				stride_y);
		for(int x = 0; x < size_o_src_x; x++)
			dwt_cdf97_f_ex_stride(
				addr2(ptr,0,x,stride_x,stride_y),
				addr2(ptr,0,x,stride_x,stride_y),
				addr2(ptr,size_o_dst_y,x,stride_x,stride_y),
				temp,
				size_i_src_y,
				stride_x);

		if(zero_padding)
		{
			for(int y = 0; y < size_o_src_y; y++)
				dwt_zero_padding_f_stride(
					addr2(ptr,y,0,stride_x,stride_y),
					addr2(ptr,y,size_o_dst_x,stride_x,stride_y),
					size_i_src_x,
					size_o_dst_x,
					size_o_src_x-size_o_dst_x,
					stride_y);
			for(int x = 0; x < size_o_src_x; x++)
				dwt_zero_padding_f_stride(
					addr2(ptr,0,x,stride_x,stride_y),
					addr2(ptr,size_o_dst_y,x,stride_x,stride_y),
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

		for(int y = 0; y < size_o_dst_y; y++)
			dwt_cdf97_i_ex_stride(
				addr2(ptr,y,0,stride_x,stride_y),
				addr2(ptr,y,size_o_src_x,stride_x,stride_y),
				addr2(ptr,y,0,stride_x,stride_y),
				temp,
				size_i_dst_x,
				stride_y);
		for(int x = 0; x < size_o_dst_x; x++)
			dwt_cdf97_i_ex_stride(
				addr2(ptr,0,x,stride_x,stride_y),
				addr2(ptr,size_o_src_y,x,stride_x,stride_y),
				addr2(ptr,0,x,stride_x,stride_y),
				temp,
				size_i_dst_y,
				stride_x);

		if(zero_padding)
		{
			for(int y = 0; y < size_o_dst_y; y++)
				dwt_zero_padding_i_stride(
					addr2(ptr,y,0,stride_x,stride_y),
					size_i_dst_x,
					size_o_dst_x,
					stride_y);
			for(int x = 0; x < size_o_dst_x; x++)
				dwt_zero_padding_i_stride(
					addr2(ptr,0,x,stride_x,stride_y),
					size_i_dst_y,
					size_o_dst_y,
					stride_x);
		}

		j--;
	}
}
