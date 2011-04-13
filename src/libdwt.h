/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Fast wavelet transform implemented via lifting scheme.
 */
#ifndef LIBDWT_H
#define LIBDWT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @deprecated Use @ref dwt_cdf97_f_ex instead.
 * @brief Lifting implementation of one level of fast wavelet transform.
 */
void dwt_cdf97_f(
	const double *src,	///< input signal of the length N
	double *dst,		///< output signal of the length N, i.e. one level of discrete wavelet transform, the L and H channels are concatenated
	double *tmp,		///< temporary memory space of the length N
	int N			///< length of the signal, odd or even length
);

/**
 * @deprecated Use @ref dwt_cdf97_i_ex instead.
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 */
void dwt_cdf97_i(
	const double *src,	///< input signal of the length N, i.e. the discrete wavelet transform
	double *dst,		///< output signal of the length N, i.e. the reconstructed signal
	double *tmp,		///< temporary memory space of the length N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform.
 */
void dwt_cdf97_f_ex(
	const double *src,	///< input signal of the length N
	double *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd N
	double *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *tmp,		///< temporary memory space of the length N
	int N			///< length of the input signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 */
void dwt_cdf97_i_ex(
	const double *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd N
	const double *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *dst,		///< reconstructed (output) signal
	double *tmp,		///< temporary memory space of the length N
	int N			///< length of the reconstructed (output) signal, odd or even length
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output.
 */
void dwt_zero_padding(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	double *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst		///< length of the L and H channels
);

#ifdef __cplusplus
}
#endif

#endif
