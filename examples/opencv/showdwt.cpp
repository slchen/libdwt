/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Example application showing variants of image fast wavelet transform.
 */
#include "libdwt.h"

#include <iostream>
#include <cmath>
#include <cv.h>
#include <highgui.h>

/**
 * This example can work with grayscale or color image.
 */
#define WITH_COLOR

using namespace std;
using namespace cv;

/**
 * Fast integer math function. Returns 1<<(int)ceil(log2(x)).
 */
static
int pow2_ceil_log2(int x)
{
	assert(x > 0);

	x = (x-1) << 1;

	int y = 0;
	while(x > 1)
	{
		x >>= 1;
		y++;
	}

	return 1 << y;
}

/**
 * Shows DWT coefficients.
 */
void wtshow(const char *name, Mat &f)
{
	Mat g(f.clone());
	log(abs(g),g);
	imshow(name, g*32/256);
}

/**
 * Resize image.
 * The image should have the number of rows and columns equal to a power of two.
 */
void resize_pow2(Mat &f)
{
	int rows = pow2_ceil_log2(f.rows);
	int cols = pow2_ceil_log2(f.cols);

	cout << "resizing (" << f.rows << "," << f.cols << ") to (" << rows << "," << cols << ")..." << endl;
	copyMakeBorder(f.clone(), f, 0, rows-f.rows, 0, cols-f.cols, BORDER_CONSTANT);
}

/**
 * Compare two images.
 */
int check(const Mat &a, const Mat &b, Size s, double eps = 1e-6)
{
	double err = norm(a(Rect(Point(0, 0), s)) - b(Rect(Point(0, 0), s)));

	cout << "error: " << err << endl;

	if( err > eps )
		return 1;
	else
		return 0;
}

#define TIMER_INIT double T
#define TIMER_START T = (double)getTickCount()
#define TIMER_PRINT cout << "time: " << ((double)getTickCount() - T)/getTickFrequency() << " secs" << endl

/**
 * Simple image DWT.
 * Size of outer frame should be power of two. Size of nested image should be smaller or equal to outer frame. Outer frame is padded with zeros. Coefficients with large aplitude appear on inner image edges. Slowest variant.
 */
void demo_simple(const Mat &src, int j = -1)
{
	Mat sqr = src.clone();

	resize_pow2(sqr);
	imshow("source", sqr/256);

	TIMER_INIT;

	TIMER_START;
	for(int c=0; c<sqr.channels(); c++)
		dwt_cdf97_2f(sqr.data+sqr.elemSize1()*c, sqr.step, sqr.elemSize(), sqr.size().width, sqr.size().height, sqr.size().width, sqr.size().height, &j, 0, 0);
	TIMER_PRINT;
	wtshow("transform", sqr);

	TIMER_START;
	for(int c=0; c<sqr.channels(); c++)
		dwt_cdf97_2i(sqr.data+sqr.elemSize1()*c, sqr.step, sqr.elemSize(), sqr.size().width, sqr.size().height, sqr.size().width, sqr.size().height, j, 0, 0);
	TIMER_PRINT;
	imshow("reconstructed", sqr/256);

	check(src, sqr, src.size());

	waitKey();
}

/**
 * Sparse image DWT.
 * Size of outer frame should be power of two. Size of nested image should be smaller or equal to outer frame. Coefficients with large aplitude do not appear on inner image edges. Undefined values remain in unused image area.
 */
void demo_sparse(const Mat &src, int j = -1)
{
	Mat sqr = src.clone();

	resize_pow2(sqr);
	imshow("source", sqr/256);

	TIMER_INIT;

	TIMER_START;
	for(int c=0; c<sqr.channels(); c++)
		dwt_cdf97_2f(sqr.data+sqr.elemSize1()*c, sqr.step, sqr.elemSize(), sqr.size().width, sqr.size().height, src.size().width, src.size().height, &j, 0, 0);
	TIMER_PRINT;
	wtshow("transform", sqr);

	TIMER_START;
	for(int c=0; c<sqr.channels(); c++)
		dwt_cdf97_2i(sqr.data+sqr.elemSize1()*c, sqr.step, sqr.elemSize(), sqr.size().width, sqr.size().height, src.size().width, src.size().height, j, 0, 0);
	TIMER_PRINT;
	imshow("reconstructed", sqr/256);

	check(src, sqr, src.size());

	waitKey();
}

/**
 * Sparse image DWT with zero padding.
 * Same as in previous case with except unused image area is filled with zeros. Slower than variant without zero padding.
 */
void demo_sparse_zeros(const Mat &src, int j = -1)
{
	Mat sqr = src.clone();

	resize_pow2(sqr);
	imshow("source", sqr/256);

	TIMER_INIT;

	TIMER_START;
	for(int c=0; c<sqr.channels(); c++)
		dwt_cdf97_2f(sqr.data+sqr.elemSize1()*c, sqr.step, sqr.elemSize(), sqr.size().width, sqr.size().height, src.size().width, src.size().height, &j, 0, 1);
	TIMER_PRINT;
	wtshow("transform", sqr);

	TIMER_START;
	for(int c=0; c<sqr.channels(); c++)
		dwt_cdf97_2i(sqr.data+sqr.elemSize1()*c, sqr.step, sqr.elemSize(), sqr.size().width, sqr.size().height, src.size().width, src.size().height, j, 0, 1);
	TIMER_PRINT;
	imshow("reconstructed", sqr/256);

	check(src, sqr, src.size());

	waitKey();
}

/**
 * Packed image DWT.
 * Outer frame size is equal to inner image size. Inner image size can be of any size. Coefficients with large aplitude do not appear on inner image edges. Fastest variant.
 */
void demo_packed(const Mat &src, int j = -1)
{
	Mat sqr = src.clone();

	imshow("source", sqr/256);

	TIMER_INIT;

	TIMER_START;
	for(int c=0; c<sqr.channels(); c++)
		dwt_cdf97_2f(sqr.data+sqr.elemSize1()*c, sqr.step, sqr.elemSize(), sqr.size().width, sqr.size().height, src.size().width, src.size().height, &j, 0, 0);
	TIMER_PRINT;
	wtshow("transform", sqr);

	TIMER_START;
	for(int c=0; c<sqr.channels(); c++)
		dwt_cdf97_2i(sqr.data+sqr.elemSize1()*c, sqr.step, sqr.elemSize(), sqr.size().width, sqr.size().height, src.size().width, src.size().height, j, 0, 0);
	TIMER_PRINT;
	imshow("reconstructed", sqr/256);

	check(src, sqr, src.size());

	waitKey();
}

int main(int argc, char **argv)
{
	Mat f;

	const char *imagename = argc > 1 ? argv[1] : "Lenna.png";
	cout << "Loading " << imagename << endl;

	imread(imagename,
#ifdef WITH_COLOR
		CV_LOAD_IMAGE_COLOR
#else
		CV_LOAD_IMAGE_GRAYSCALE
#endif
	).convertTo(f, CV_64F);

	if(!f.data)
	{
		cerr << "Unable to load input image" << endl;
		return -1;
	}

	demo_simple(f, 2);

	demo_simple(f);

	demo_sparse(f);

	demo_sparse_zeros(f);

	demo_packed(f);

	return 0;
}
