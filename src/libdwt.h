#ifndef LIBDWT_H
#define LIBDWT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Lifting implementation of one level of fast wavelet transform.
 */
void dwt_cdf97_f(
	const double *src,	///< input signal of the length N
	double *dst,		///< output signal of the length N
	double *tmp,		///< temporary memory space of the length N
	int N			///< length of the signal
);

/**
 * Lifting implementation of one level of fast inverse wavelet transform.
 */
void dwt_cdf97_i(
	const double *src,	///< input signal of the length N
	double *dst,		///< output signal of the length N
	double *tmp,		///< temporary memory space of the length N
	int N			///< length of the signal
);

#ifdef __cplusplus
}
#endif

#endif
