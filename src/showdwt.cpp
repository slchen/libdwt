#include "libdwt.h"

#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

/**
 * decompozition up to 2 coefficients, note that decomposition of 1 coeficient does not make any sense, so set to 2 for full decomposition
 */
static const int min_rows = 2;

/**
 * forward image DWT using fwt97 function
 */
void fwt(Mat_<double> &f)
{
	Mat_<double> t(Size(f.rows,1));

	for(int rows = f.rows; rows >= min_rows; rows>>=1)
	{
		for(int y = 0; y < rows; y++)
			dwt_cdf97_f(f.ptr<double>(y), f.ptr<double>(y), t.ptr<double>(0), rows);
		transpose(f,f);
		for(int y = 0; y < rows; y++)
			dwt_cdf97_f(f.ptr<double>(y), f.ptr<double>(y), t.ptr<double>(0), rows);
		transpose(f,f);
	}
}

/**
 * inverse image DWT using iwt97
 */
void iwt(Mat_<double> &f)
{
	Mat_<double> t(Size(f.rows,1));

	for(int rows = min_rows; rows <= f.rows; rows<<=1)
	{
		for(int y = 0; y < rows; y++)
			dwt_cdf97_i(f.ptr<double>(y), f.ptr<double>(y), t.ptr<double>(0), rows);
		transpose(f,f);
		for(int y = 0; y < rows; y++)
			dwt_cdf97_i(f.ptr<double>(y), f.ptr<double>(y), t.ptr<double>(0), rows);
		transpose(f,f);
	}
}

/**
 * image should have the same number of rows like columns and these must be a power of two
 */
void resize(Mat_<double> &f)
{
	int rows = 1<<(int)ceil(log(f.rows)/log(2));
	int cols = 1<<(int)ceil(log(f.cols)/log(2));
	int size = max(rows, cols);

	if( f.rows == size && f.cols == size)
	{
		cout << "good size (" << f.rows << "," << f.cols << "), no resize needed" << endl;
	}
	else
	{
		cout << "resizing (" << f.rows << "," << f.cols << ") to (" << size << "," << size << ")..." << endl;
		copyMakeBorder(f.clone(), f, 0, size-f.rows, 0, size-f.cols, BORDER_CONSTANT);
	}

	// not necessary
	if( (f.rows != f.cols) || !(f.rows && !( (f.rows-1) & f.rows )) )
		CV_Error_(CV_StsOutOfRange, ("the matrix size (%d,%d) have to be power of 2 and number of rows have to be equal to number of columns", f.rows, f.cols));
}

/**
 * helper function to show DWT coefficients
 */
void show_dwt(const char *name, Mat_<double> &f)
{
	Mat_<double> g(f.clone());
	log(abs(g),g);
	imshow (name, Mat_<uchar>(g*32));
}

int main(int argc, char** argv)
{
	const char *imagename = argc > 1
		? argv[1] : "Lenna.png";
	cout << "Loading " << imagename << endl;

	 Mat_<double> f = Mat_<double>(imread(imagename, CV_LOAD_IMAGE_GRAYSCALE));
	if(!f.data)
		return -1;

	::resize(f);
	imshow("in", Mat_<uchar>(f));
	fwt(f);
	show_dwt("dwt", f);
	iwt(f);
	imshow("out", Mat_<uchar>(f));
	waitKey();

	return 0;
}
