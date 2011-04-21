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
 * @brief Lifting implementation of one level of fast wavelet transform.
 * @deprecated Use @ref dwt_cdf97_f_ex instead.
 */
void dwt_cdf97_f(
	const double *src,	///< input signal of the length @e N
	double *dst,		///< output signal of the length @e N, i.e. one level of discrete wavelet transform, the L and H channels are concatenated
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 * @deprecated Use @ref dwt_cdf97_i_ex instead.
 */
void dwt_cdf97_i(
	const double *src,	///< input signal of the length @e N, i.e. the discrete wavelet transform
	double *dst,		///< output signal of the length @e N, i.e. the reconstructed signal
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform.
 */
void dwt_cdf97_f_ex(
	const double *src,	///< input signal of the length @e N
	double *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	double *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the input signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform performed on column.
 * @warning Experimental.
 */
void dwt_cdf97_f_ex_stride(
	const double *src,	///< input signal of the length @e N
	double *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	double *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the input signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e src, @e dst_l and @e dst_h
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 */
void dwt_cdf97_i_ex(
	const double *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const double *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *dst,		///< reconstructed (output) signal
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the reconstructed (output) signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform.
 * @warning Experimental.
 */
void dwt_cdf97_i_ex_stride(
	const double *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const double *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *dst,		///< reconstructed (output) signal
	double *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the reconstructed (output) signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst, @e src_l and @e src_h
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 */
void dwt_zero_padding_f(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	double *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H		///< length of the H channel
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 * @warning Experimental.
 */
void dwt_zero_padding_f_stride(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	double *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H,		///< length of the H channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst_l and @e dst_h
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 */
void dwt_zero_padding_i(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst		///< total length of the L channel
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 * @warning Experimental.
 */
void dwt_zero_padding_i_stride(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst,		///< total length of the L channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels
);

/**
 * @brief Forward image fast wavelet transform using CDF 9/7 wavelet and lifting scheme, in-place version.
 * @warning Experimental.
 */
void dwt_cdf97_2f(
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
 * @warning Experimental.
 */
void dwt_cdf97_2i(
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

#ifdef __cplusplus
}
#endif

#endif
