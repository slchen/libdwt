#include "cvdwt.h"

#include <cmath> // ceil log2; FIXME: -lm
#include <highgui.h> // imshow

using namespace cv;
using namespace dwt;

void dwt::resizePOT(
	Mat &img,
	int borderType)
{
	const int rows = 1<<(int)ceil(log2(img.rows)); // FIXME: faster implementation
	const int cols = 1<<(int)ceil(log2(img.cols)); // FIXME: faster implementation

	copyMakeBorder(img.clone(), img, 0, rows-img.rows, 0, cols-img.cols, borderType);
}

void dwt::resizePOT(
	const Mat &src,
	Mat &dst,
	int borderType)
{
	const int rows = 1<<(int)ceil(log2(src.rows)); // FIXME: faster implementation
	const int cols = 1<<(int)ceil(log2(src.cols)); // FIXME: faster implementation

	copyMakeBorder(src, dst, 0, rows-src.rows, 0, cols-src.cols, borderType);
}

int dwt::isPOT(
	const Mat &img)
{
	const int rows = 1<<(int)ceil(log2(img.rows)); // FIXME: faster implementation
	const int cols = 1<<(int)ceil(log2(img.cols)); // FIXME: faster implementation

	return (img.rows == rows) && (img.cols == cols);
}

void dwt::wtshow(
	const string &winname,
	const Mat &image)
{
	double a = 100;
	double b = 10;

	Mat g;

	log(1+abs(image)*a, g);

	imshow(winname, g/b);
}

static
int is_set(int word, int flag)
{
	return word&flag ? 1 : 0;
}

static
void cv_dwt_cdf97_2f(Mat &img, const int &channel, const Size &size, int &j, const int &flags)
{
	switch(img.depth())
	{
		case CV_64F:
			dwt_cdf97_2f_d(
				img.data+img.elemSize1()*channel,
				img.step,
				img.elemSize(),
				img.size().width,
				img.size().height,
				size.width,
				size.height,
				&j,
				is_set(flags, DWT_EXTREME),
				is_set(flags, DWT_PADDING));
			break;
		case CV_32F:
			dwt_cdf97_2f_s(
				img.data+img.elemSize1()*channel,
				img.step,
				img.elemSize(),
				img.size().width,
				img.size().height,
				size.width,
				size.height,
				&j,
				is_set(flags, DWT_EXTREME),
				is_set(flags, DWT_PADDING));
			break;	
		default:
			CV_Error(CV_StsOutOfRange, "The matrix element depth value is out of range.");
	}
}

static
void cv_dwt_cdf97_2i(Mat &img, const int &channel, const Size &size, const int &j, const int &flags)
{
	switch(img.depth())
	{
		case CV_64F:
			dwt_cdf97_2i_d(
				img.data+img.elemSize1()*channel,
				img.step,
				img.elemSize(),
				img.size().width,
				img.size().height,
				size.width,
				size.height,
				j,
				is_set(flags, DWT_EXTREME),
				is_set(flags, DWT_PADDING));
			break;
		case CV_32F:
			dwt_cdf97_2i_s(
				img.data+img.elemSize1()*channel,
				img.step,
				img.elemSize(),
				img.size().width,
				img.size().height,
				size.width,
				size.height,
				j,
				is_set(flags, DWT_EXTREME),
				is_set(flags, DWT_PADDING));
			break;	
		default:
			CV_Error(CV_StsOutOfRange, "The matrix element depth value is out of range.");
	}
}

void dwt::transform(
	Mat &img,
	Size size,
	int &j,
	int flags)
{
	CV_Assert( img.depth() == CV_64F || img.depth() == CV_32F );
	CV_Assert( 1 == is_set(flags, DWT_FORWARD) + is_set(flags, DWT_INVERSE) );
	CV_Assert( 1 == is_set(flags, DWT_SIMPLE) + is_set(flags, DWT_SPARSE) + is_set(flags, DWT_PACKED) );

	if( is_set(flags,DWT_FORWARD) )
	{
		if( is_set(flags, DWT_SIMPLE) || is_set(flags, DWT_SPARSE) )
			resizePOT(img);
	
		if( is_set(flags, DWT_SIMPLE) || is_set(flags, DWT_PACKED) )
			size = img.size();

		for(int c = 0; c < img.channels(); c++)
			cv_dwt_cdf97_2f(img, c, size, j, flags);
	}
	else
	{
		if( is_set(flags, DWT_SIMPLE) || is_set(flags, DWT_SPARSE) )
			CV_Assert( isPOT(img) );

		if( is_set(flags, DWT_SIMPLE) || is_set(flags, DWT_PACKED) )
			size = img.size();

		for(int c = 0; c < img.channels(); c++)
			cv_dwt_cdf97_2i(img, c, size, j, flags);
	}
}

void dwt::transform(
	const Mat &src,
	Mat &dst,
	Size size,
	int &j,
	int flags)
{
	if(src.data == dst.data)
		dst = src; // no data is copied
	else
		dst = src.clone(); // full copy

	transform(
		dst,
		size,
		j,
		flags);
}
