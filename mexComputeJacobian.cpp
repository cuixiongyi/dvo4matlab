#include <Engine.h>
#include <vector>
/*
#pragma comment (lib, "libmat.lib")
#pragma comment (lib, "libeng.lib")
#pragma comment (lib, "libmex.lib")
#pragma comment (lib, "libmx.lib")
*/

#include <math.h>
#include "mex.h"
#define IS_REAL_2D_FULL_DOUBLE(P) (!mxIsComplex(P) && mxGetNumberOfDimensions(P) == 2 && !mxIsSparse(P) && mxIsDouble(P))
#define IS_REAL_SCALAR(P) (IS_REAL_2D_FULL_DOUBLE(P) && mxGetNumberOfElements(P) == 1)
long length(0);
int width(0), height(0);

mxArray * getMexArray(const std::vector<double>& v, const int M, const int N)
{
	mxArray * mx = mxCreateDoubleMatrix(M, N, mxREAL);
	std::copy(v.begin(), v.begin()+M*N, mxGetPr(mx));
	return mx;
}

inline void computeJacobianEach(const float& x, const float& y, const float& z, const double* xpose, const float& dx, const float& dy, const double* intrinsic, std::vector<double>& j)
{
	double v1 = xpose[0];
	double v2 = xpose[1];
	double v3 = xpose[2];
	double w1 = xpose[3];
	double w2 = xpose[4];
	double w3 = xpose[5];
	double w1sq = w1*w1;
	double w2sq = w2*w2;
	double w3sq = w3*w3;
	double w1w3 = w1*w3;
	double w1w2 = w1*w2;
	double w2w3 = w2*w3;
	double fx_ = intrinsic[0];
	double fy_ = intrinsic[1];
	double cx_ = intrinsic[2];
	double cy_ = intrinsic[3];

	double e1 = (2 * v3 - z*(w1sq + w2sq - 2) - x*(2 * w2 - w1w3) + y*(2 * w1 + w2w3)); e1 = e1*e1;
	double e2 = v1 - x*(w2sq + w3sq - 2) / 2 - y*(2 * w3 - w1w2) / 2 + z*(2 * w2 + w1w3) / 2;
	double e3 = v2 - y*(w1sq + w3sq - 2) / 2 + x*(2 * w3 + w1w2) / 2 - z*(2 * w1 - w2w3) / 2;
	double e4 = x - w3*y / 2 + w2*z - w2sq*x / 2 + w1w2*y / 2;
	double e5 = y + w3*x / 2 - w1*z - w1sq*y / 2 + w1w2*x / 2;
	double e6 = w1w2*z;

	double e10 = w2sq*z;
	double e11 = w1sq*z;
	double e12 = 2 * w1*y;
	double e13 = 2 * w2*x;
	double e14 = w2w3*y;
	double e15 = w1w3*x;
	double e16 = w1sq / 2 + w2sq / 2 + w3sq / 2;
	double e17 = w1sq + w2sq + w3sq;
	double e18 = w1w3*y;
	double e19 = w2w3*x;

	double e7 = 2 * z + 2 * v3 - e13 + e12 - e11 - e10 + e15 + e14;
	double e8 = v3 - z*((w1sq + w2sq)*e16 / e17 - 1) - x*(w2 - w1w3*e16 / e17) + y*(w1 + w2w3*e16 / e17);
	double e9 = w1*x / 2 + w2*y / 2 + e19 / 2 - e18 / 2;

	j.clear();
	j.push_back(dx*fx_ / e8);
	j.push_back(dy*fy_ / e8);
	j.push_back(-4*dx*fx_*e2/e1 - 4*dy*fy_*e3/e1 - 1);
	j.push_back(y*(w1sq - 2) / 2 - x*(w3 + w1w2) / 2 + w1*z + dx*(fx_*(w2*y + w3*z + e18 - e16) / e7 - fx_*e2*e5 * 4 / e1) - dy*(fy_*(2*z-w2*x+e12-e11+e15)/e7+(fy_*e3*e5*4)/e1));
	j.push_back(w2*z - x*(w2sq - 2) / 2 - y*(w3 - w1w2) / 2 + dy*(fy_*(w1*x + w3*z - e19 + e6) / e7 + fy_*e3*e4 * 4 / e1) + dx*(fx_*(2 * z - e13 + w1*y - e10 + e14) / e7 + fx_*e2*e4 * 4 / e1));
	j.push_back(dy*(fy_*(2*x-2*w3*y+w2*z-w3sq*x+w1w3*z)/e7-fy_*e9*e3*4/e1)-dx*(fx_*(2*y+2*w3*x-w1*z-w3sq*y+w2w3*z)/e7+fx_*e9*e2*4/e1)-x*(w1+w2w3)/2-y*(w2-w1w3*2));







	return;
}

// input 1, residual, residualCorres, pointcloud1X,pointcloudY,pointcloudZ, xpose, devrative2x, devrative2y, weight, intrinsic(fx_, fy_, cx_, cy_)
// output 1, n*6
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	/* Macros for the ouput and input arguments */
#define DoutJacobian plhs[0]
#define DoutA plhs[1]
#define Doutb plhs[2]

#define Dresidual prhs[0]
#define DresidualCorres prhs[1]
#define Dpointcloud1X prhs[2]
#define DpointcloudY prhs[3]
#define DpointcloudZ prhs[4]
#define Dxpose prhs[5]
#define Ddevrative2x prhs[6]
#define Ddevrative2y prhs[7]
#define Dweight prhs[8]
#define Dintrinsic prhs[9]


	const float* residual((float*)mxGetData(prhs[0]));
	const unsigned __int16* residualCorres((unsigned __int16*)mxGetData(prhs[1]));
	const float* pcX((float*)mxGetData(prhs[2]));
	const float* pcY((float*)mxGetData(prhs[3]));
	const float* pcZ((float*)mxGetData(prhs[4]));
	const double* xpose(mxGetPr(prhs[5]));
	const float* dx((float*)mxGetData(prhs[6]));
	const float* dy((float*)mxGetData(prhs[7]));
	const double* weight(mxGetPr(prhs[8]));
	const double* intrinsic(mxGetPr(prhs[9]));

	std::vector<double> outJacobian;
//	std::vector<double> outA;
//	std::vector<double> outb;
	
	const long mResidual = mxGetM(Dresidual);
	const int nResidual = mxGetN(Dresidual);
	const int mpointcloud1X = mxGetM(Dpointcloud1X);
	const int npointcloud1X = mxGetN(Dpointcloud1X);
	width = npointcloud1X;
	height = mpointcloud1X;
	length = mResidual;
	outJacobian.resize(length* 6);
//	outA.resize(6 * 6);
//	outb.resize(6);
	//B_OUT = mxCreateDoubleMatrix(length, 6, mxREAL); /* Create the output matrix */
	
	std::vector<double> j;
	for (int ii = 0; ii < length; ++ii)
	{
		int yy = residualCorres[ii]-1;
		int xx = residualCorres[ii + length]-1;
		computeJacobianEach(pcX[yy + xx*height], pcY[yy + xx*height], pcZ[yy + xx*height], xpose, dx[yy + xx*height], dy[yy + xx*height], intrinsic, j);
		for (int jj = 0; jj < 6;++jj)
			outJacobian[ii + length * jj] = j[jj];
	}
	
	/*for (int m = 0; m < length; m++)
	{
		for (int n = 0; n < 6; n++)
			outJacobian[m + length*n] = 1024;
	}
	*/
	DoutJacobian = getMexArray(outJacobian, length, 6);
//	DoutA = getMexArray(outA, 6, 6);
//	Doutb = getMexArray(outb, 6, 1);
	return;
}