# Compiling #
First, download and unpack library source codes. Then, compile the libdwt static library like follows.

```
make ARCH=x86_64 BUILD=release -C src libdwt.a
```

Currently, following architectures are supported.

| **architecture** | **description** |
|:-----------------|:----------------|
| `x86_64`         | AMD64, EM64T    |
| `microblaze`     | MicroBlaze core in Xilinx FPGA |
| `asvp`           | UTIA's ASVP platform |
| `armv6l`         | ARM11 family, e.g. Raspberry Pi |
| `armv7l`         | Cortex-A8 family, e.g. Nokia N900 |

Now, you can use the library in your program (include `libdwt.h` and link against `libdwt.a`).
Finally, do not forget to link your application also with `-lm`, `-lrt`, and enable OpenMP with `-fopenmp`.
Further, look at `example` directory for understanding the use of this library.

# Simple program #

Include `libdwt.h` header file and call `dwt_util_init` and `dwt_util_finish` functions that allocate and release resources of the library.

```
#include "libdwt.h"

int main()
{
	dwt_util_init();

	// your code here

	dwt_util_finish();

	return 0;
}
```

# Image allocation #

Use `dwt_util_alloc_image` function to allocate a memory for your images. The transform is computed in-place. Thus, you do not need any extra memory allocated for handling of transform. Here, we will use `float` data type. Every function handling this data type has `_s` suffix. Furthermore, we consider a single-channel image. So, the distance between subsequent columns equals to `sizeof(float)`. Conversely distance between subsequent lines should be computed using `dwt_util_get_opt_stride` function. When you finish with your work, the allocated memory should be freed with `dwt_util_free_image` function call.

```
// image sizes
const int x = 1920, y = 1080;

// image strides
const int stride_y = sizeof(float);
const int stride_x = dwt_util_get_opt_stride(x * sizeof(float));

// image data
void *data;

dwt_util_alloc_image(&data, stride_x, stride_y, x, y);
```

# Performing transform #

In this section, we focus on CDF 9/7 wavelet used in the [JPEG 2000](http://www.jpeg.org/jpeg2000/) image compression standard. Moreover, we want one level of decomposition (LL1, HL1, LH1 and HH1 sub-bands). The 2-D forward transform can be performed by `dwt_cdf97_2f_s` function. Analogously, the inverse transform can be performed using `dwt_cdf97_2i_s` with same parameters. Both functions work in-place.

```
// level of decomposition
int j = 1;

// forward transform
dwt_cdf97_2f_s(data, stride_x, stride_y, x, y, x, y, &j, 0, 0);

// process the transform here

// inverse transform
dwt_cdf97_2i_s(data, stride_x, stride_y, x, y, x, y, j, 0, 0);
```

# Sub-band access #

On each level of decomposition (on each scale), we obtain four sub-bands which correspond to the image approximation and its horizontal, vertical and diagonal edges.
Sub-band with image approximation (LL) is preserved only on the highest level of image decomposition (the coarsest scale).
Thus, for each scale we can obtain three sub-bands (HL, LH and HH).
Finally, at the highest decomposition level we get all four sub-bands (LL, HL, LH and HH).
To access to the coefficients in a particular sub-band at a certain scale, you can use the `dwt_util_subband_s` function.
Through the parameters of this function, we pass a level of decomposition and a sub-band identifier (either `DWT_LL`, `DWT_HL`, `DWT_LH` or `DWT_HH`).
The function will return a sub-band dimensions and a pointer to its data.

```
void *subband_ptr;
int subband_size_x;
int subband_size_y;

dwt_util_subband_s(data, stride_x, stride_y, x, y, x, y, j, DWT_LH, &subband_ptr, &subband_size_x, &subband_size_y);

```

# Pixel access #

The address of any coefficient (or pixel) can be computed using `dwt_util_addr_coeff_s` function. Feel free to change a value at the obtained address.

```
float *coeff = dwt_util_addr_coeff_s(data, y, x, stride_x, stride_y);
```

# Data types #
The libdwt library currently supports single precision floating point type (`single`), double precision floating point type (`double`) and integer type (`int`). The corresponding functions have suffixes `_s` (single precision FP), `_d` (double precision FP) and `_i` (integer). The situation is illustrated in the following table.

| **transform** | **float** | **double** | **int** |
|:--------------|:----------|:-----------|:--------|
| CDF 5/3       | yes       | yes        | yes     |
| CDF 9/7       | yes<sup>*</sup>  | yes        | yes     |

Notes:
<br>
<sup>*</sup> accelerated with SSE (PC platform) or BCE (ASVP platform)<br>
<br>
<h1>Feature description</h1>
Once the image is transformed into discrete wavelet transform, the feature vector can be extracted using one of the following aggregate functions. Vector elements correspond to the subbands.<br>
<br>
<ul><li><code>dwt_util_wps_s</code> – the rectified wavelet power spectra<br>
</li><li><code>dwt_util_maxidx_s</code> – the indices of coefficients with maximal magnitudes<br>
</li><li><code>dwt_util_mean_s</code> – the arithmetic means<br>
</li><li><code>dwt_util_med_s</code> – the medians<br>
</li><li><code>dwt_util_var_s</code> – the variances<br>
</li><li><code>dwt_util_stdev_s</code> – the standard deviations<br>
</li><li><code>dwt_util_skew_s</code> – the skewnesses<br>
</li><li><code>dwt_util_kurt_s</code> – the kurtosises<br>
</li><li><code>dwt_util_maxnorm_s</code> – the maximum norms<br>
</li><li><code>dwt_util_lpnorm_s</code> – the p-norms<br>
</li><li><code>dwt_util_norm_s</code> – the Euclidean norms