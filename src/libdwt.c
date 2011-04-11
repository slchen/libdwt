#include "libdwt.h"

/**
 * @{
 * These constants are found in S. Mallat. A Wavelet Tour of Signal Processing: The Sparse Way (Third Edition). 3rd edition, 2009 on page 370.
 */
static const double dwt_cdf97_p1 =  1.58613434342059;
static const double dwt_cdf97_u1 = -0.0529801185729;
static const double dwt_cdf97_p2 = -0.8829110755309;
static const double dwt_cdf97_u2 =  0.4435068520439;
static const double dwt_cdf97_s1  =   1.1496043988602;
static const double dwt_cdf97_s2  = 1/1.1496043988602;
/**@}*/

/**
 * Lifting implementation of one level of fast wavelet transform.
 */
void dwt_cdf97_f(
	const double *src,	///< input signal of the length N
	double *dst,		///< output signal of the length N
	double *tmp,		///< temporary memory space of the length N
	int N			///< length of the signal
)
{
	// copy src into tmp
	for(int i=0; i<N; i++)
		tmp[i] = src[i];

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

	// scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s1;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s2;

	// copy tmp into dst
	for(int i=0; i<N; i+=2)
		dst[    i/2] = tmp[i];
	for(int i=1; i<N; i+=2)
		dst[N/2+i/2] = tmp[i];
}

/**
 * Lifting implementation of one level of fast inverse wavelet transform.
 */
void dwt_cdf97_i(
	const double *src,	///< input signal of the length N
	double *dst,		///< output signal of the length N
	double *tmp,		///< temporary memory space of the length N
	int N			///< length of the signal
)
{
	// copy src into tmp
	for(int i=0; i<N/2; i++)
	{
		tmp[2*i  ] = src[    i];
		tmp[2*i+1] = src[N/2+i];
	}

	// inverse scale
	for(int i=0; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s2;
	for(int i=1; i<N; i+=2)
		tmp[i] = tmp[i] * dwt_cdf97_s1;

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

	// copy tmp into dst
	for(int i=0; i<N; i++)
		dst[i] = tmp[i];
}
