#include "libdwt.h"

#include <assert.h>
#include <stddef.h>

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
	assert(!( N < 0 || NULL == src || NULL == dst_l || NULL == dst_h || NULL == tmp ));

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst_l[0] = src[0] * dwt_cdf97_s1;
		return;
	}

	// copy src into tmp
	for(int i=0; i<N; i++)
		tmp[i] = src[i];

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
		dst_l[i] = tmp[2*i  ];
	for(int i=0; i<N/2      ; i++)
		dst_h[i] = tmp[2*i+1];
}

void dwt_cdf97_i_ex(
	const double *src_l,
	const double *src_h,
	double *dst,
	double *tmp,
	int N
)
{
	assert(!( N < 0 || NULL == src_l || NULL == src_h || NULL == dst || NULL == tmp ));

	// fix for small N
	if(N < 2)
	{
		if(1 == N)
			dst[0] = src_l[0] * dwt_cdf97_s2;
		return;
	}

	// copy src into tmp
	for(int i=0; i<N/2+(N&1); i++)
		tmp[2*i  ] = src_l[i];
	for(int i=0; i<N/2      ; i++)
		tmp[2*i+1] = src_h[i];

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
		dst[i] = tmp[i];
}

void dwt_zero_padding(
	double *dst_l,
	double *dst_h,
	int N,
	int N_dst
)
{
	assert(!( N < 0 || N_dst < 0 || NULL == dst_l || NULL == dst_h ));

	if(N_dst)
	{
		for(int i=N/2+(N&1); i<N_dst; i++)
			dst_l[i] = dst_h[i] = 0;
	
		if((N&1) && N/2<N_dst)
			dst_h[N/2] = 0;
	}
}
