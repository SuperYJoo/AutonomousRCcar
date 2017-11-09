
#include "stdafx.h"
#include <iostream>
#include<opencv/cvaux.h>
#include<opencv/highgui.h>
#include<opencv/cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<stdio.h>
#include<stdlib.h>
#include <vector>
// Need to include this for serial port communication
#include <Windows.h>
#include <math.h>
#include <assert.h>
#include <fstream>

#define	CROPPING_HEIGHT 269
#define ANGLE_THRESHOLD 160

#define M_PI		3.14159265358979323846	// pi 
#define M_PIf		3.14159265358979323846f	// pi 

using namespace cv;
using namespace std;

ofstream fout;

struct sPoint {
	int x, y;

	sPoint() : x(0), y(0) { }
	sPoint(int x_, int y_) : x(x_), y(y_) { }
};


struct sLine {
	int sx, sy;
	int ex, ey;
	int dx, dy;
	double len;


	sLine() : sx(0), sy(0), ex(0), ey(0), dx(0), dy(0), len(0.) { }
	sLine(int sx_, int sy_, int ex_, int ey_)
		: sx(sx_), sy(sy_), ex(ex_), ey(ey_)
	{
		dx = ex - sx;
		dy = ey - sy;
		len = sqrt((double)(dx*dx + dy*dy));
	}
};

void cvDetectColor(IplImage *src, IplImage *dst);
void cvDetectLine(IplImage* org, IplImage* src, IplImage* dst);
void LaneCandidate(IplImage *mag, IplImage *ori, IplImage *dst);
sPoint VanishingPoint(vector<sLine> &cand, vector<sLine> *inlier);
double ransac_vanishing_point(vector<sLine> &data, sPoint &model, double distance_threshold);
bool find_in_samples(const vector<sLine> &samples, const sLine &data);
vector<sLine> get_samples(int no_samples, const vector<sLine> &data);
//sPoint compute_model_parameter(const vector<sLine> &samples);
double compute_distance(const sLine &line, const sPoint &x);
double model_verification(vector<sLine> &inliers, const sPoint &model, const vector<sLine> &data, double distance_threshold);
inline double DeltaRad(double ang1, double ang2);
inline int INT_(const double a);
void Draw_Line(Mat* input_Matrix_1, Mat* input_Matrix_2, vector<Vec2f> input_vector);
double getRotationAngle(const IplImage* src);

//ofstream fout;

IplImage *draw = NULL;
sPoint _vp;

int main(int argc, char* argv[])

{
	// Setup OpenCV variables and structures
	CvSize size640x480 = cvSize(640, 480);   // use a 640 x 480 size for all windows, also make sure your webcam is set to 640x480 !!
	CvCapture* p_capWebcam;      // we will assign our web cam video stream to this later . . .
	IplImage* p_imgOriginal;   // pointer to an image structure, this will be the input image from webcam
	IplImage* p_imgProcessed;   // pointer to an image structure, this will be the processed image
	IplImage* p_imgProcessed2;
	IplImage* p_imgHSV;
	CvMemStorage* p_strStorage;   // necessary storage variable to pass into cvHoughCircles()
	char charCheckForEscKey;   // char for checking key press (Esc exits program)
	double line_degree = 0;

							   //p_capWebcam = cvCaptureFromCAM(0); // 0 => use 1st webcam, may have to change to a different number if you have multiple cameras
							   //C++ API �� �̿�  �������
	VideoCapture vc(0); //0����ķ �ʱ�ȭ
	if (!vc.isOpened())
	{
		printf("�������� ������ �� �����ϴ�!\n");
		return 0; // �������
	}
	

	// declare 1 windows
	cvNamedWindow("Original", CV_WINDOW_AUTOSIZE);  // original image from webcam


	p_imgHSV = cvCreateImage(size640x480, IPL_DEPTH_8U, 3);

	// Main program loop

	
	Mat img; //img����
	while (1) {        // for each frame . . .

		fout.open("test.txt", ios::trunc);
		line_degree = 0; // ��Űȭ
		vc >> img; //0����ķ���� ���� �����͸� img�� ����

		p_imgOriginal = &IplImage(img);  //<<--����ȯ Ű����Ʈ
		p_imgProcessed = cvCreateImage(cvGetSize(p_imgOriginal), IPL_DEPTH_8U, 1);
		p_imgProcessed2 = cvCreateImage(cvGetSize(p_imgOriginal), IPL_DEPTH_8U, 1);


		if (p_imgOriginal == NULL) {     // if frame was not captured successfully . . .
			printf("error: frame is NULL \n");  // error message to std out
			getchar();
			break;
		}

		cvCvtColor(p_imgOriginal, p_imgHSV, CV_BGR2HSV);

		cvDetectColor(p_imgOriginal, p_imgProcessed); // �� ������ ã�´�.

													
		draw = p_imgProcessed2;

		// 0 ~ 255 �׷��� �����Ϸ� ��ȯ�Ѵ�.
		IplImage* gray = cvCreateImage(cvGetSize(p_imgOriginal), IPL_DEPTH_8U, 1);
		cvCvtColor(p_imgOriginal, gray, CV_RGB2GRAY);

		// 0. ~ 1. �׷��� �����Ϸ� ��ȯ�Ѵ�.
		IplImage *img_32f = cvCreateImage(cvGetSize(gray), IPL_DEPTH_32F, 1);
		cvConvertScale(gray, img_32f, 1.0 / 255.0, 0);

		cvSmooth(img_32f, img_32f, CV_GAUSSIAN, 5);

		// Sobel �����ڷ� �̹����� x, y �������� �̺��Ѵ�. ��, edge ����.
		IplImage *diff_x = cvCreateImage(cvGetSize(img_32f), IPL_DEPTH_32F, 1);
		IplImage *diff_y = cvCreateImage(cvGetSize(img_32f), IPL_DEPTH_32F, 1);
		cvSobel(img_32f, diff_x, 1, 0, 3);
		cvSobel(img_32f, diff_y, 0, 1, 3);

		// Edge�� magnitude�� orientation�� ����Ѵ�.
		IplImage *mag = cvCreateImage(cvGetSize(img_32f), IPL_DEPTH_32F, 1);
		IplImage *ori = cvCreateImage(cvGetSize(img_32f), IPL_DEPTH_32F, 1);
		cvCartToPolar(diff_x, diff_y, mag, ori, 0);

		// Lane �ĺ� ������ ����� �̹����� �����.
		IplImage *mag2 = cvCreateImage(cvGetSize(mag), IPL_DEPTH_8U, 1);
		cvZero(mag2);

		// Lane�� �ĺ��� �� ������ ã�´�.
		LaneCandidate(mag, ori, mag2);

		// Lane �ĺ� ������ ������ ���� Ȯ��
		vector<sLine> cand;
		cand.reserve(1000);

		CvMemStorage* storage = cvCreateMemStorage(0);

		// cvHoughLines2
		// threshold - A line is returned by the function if the corresponding accumulator value is greater than threshold
		// param1 - minimum line length
		// param2 - maximum gap between line segments lying on the same line to treat them as a single line segment
		CvSeq *lines = cvHoughLines2(mag2, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 5, 2, 10);
		for (int i = 0; i < lines->total; i++) {
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);

			cand.push_back(sLine(line[0].x, line[0].y, line[1].x, line[1].y));

			// For Debugging
			cvLine(draw, line[0], line[1], CV_RGB(255, 0, 0), 3, 8);
		}

		cvReleaseMemStorage(&storage);

		// Lane�� �ĺ� ����κ��� �ҽ���(vanishing point)�� ����ؼ�,
		// �� �ҽ����� ���� inlier ������ �ɷ�����.
		vector<sLine> inlier;
		sPoint vp = VanishingPoint(cand, &inlier);

		vector<Vec2f> lines_vector;

		Mat Line_Matrix(cvGetSize(p_imgOriginal), CV_8UC1, Scalar(255));
		Mat File_Matrix(cvGetSize(p_imgProcessed2), CV_8UC1, Scalar(255));

		Mat* pLine_Matrix = &Line_Matrix;
		Mat* pFile_Matrix = &File_Matrix;

		// �ҽ��� ǥ��
		cvCircle(p_imgProcessed2, cvPoint(vp.x, vp.y), 5, CV_RGB(0, 255, 0), 3, 8);
		
		cvDetectLine(p_imgOriginal, p_imgProcessed, p_imgProcessed2);
		//Draw_Line(pLine_Matrix, pFile_Matrix, lines_vector);

		// Lane�� �ĺ� ���� ǥ��
		for (unsigned int i = 0; i<inlier.size(); i++) {
			cvLine(draw, cvPoint(inlier[i].sx, inlier[i].sy), cvPoint(inlier[i].ex, inlier[i].ey), CV_RGB(0, 255, 0), 3, 8);
			//cout << "�ҽ��� : " << inlier[i].sx << " , " << inlier[i].sy << endl;
		}
		
		cvReleaseImage(&gray);
		cvReleaseImage(&img_32f);
		cvReleaseImage(&diff_x);
		cvReleaseImage(&diff_y);
		cvReleaseImage(&mag2);
		cvReleaseImage(&mag);
		cvReleaseImage(&ori);

		line_degree = getRotationAngle(p_imgProcessed2);
		cout << "���� �� ������ ���� : " << line_degree << endl;
		
		if (line_degree > 84 || line_degree < 6)
			fout << 2 << endl; // ������ ��ȣ�� �Է�
		else if (line_degree > 6 && line_degree < 50)
			fout << 1 << endl; // ��ȸ�� ��ȣ�� �Է�
		else
			fout << 3 << endl; // ��ȸ�� ��ȣ�� �Է�

		cvShowImage("result", p_imgProcessed2);
		cvShowImage("Original", p_imgOriginal);   // original image with detectec ball overlay

		charCheckForEscKey = cvWaitKey(10);    // delay (in ms), and get key press, if any
		if (charCheckForEscKey == 27) break;    // if Esc key (ASCII 27) was pressed, jump out of while loop
		fout.close();

	} // end while
	

	  //cvReleaseCapture(&p_capWebcam);     // release memory as applicable
	cvDestroyWindow("Original");
	//destroyAllWindows();
	

	return(0);
}


void cvDetectColor(IplImage *src, IplImage *dst)
{
	int i, j;
	IplImage *r = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	IplImage *g = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	IplImage *b = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);



	cvSplit(src, b, g, r, NULL); // ���� ������ B, G, R �� ä���� �и��Ͽ� ����

	for (i = 0; i < src->height; i++)
	{
		for (j = 0; j < src->width; j++) // ��� ���� ����. �ش� ������ �������.
		{
			if ((unsigned char)g->imageData[i*g->widthStep + j] >= 220 &&
				(unsigned char)g->imageData[i*g->widthStep + j] <= 255 &&
				(unsigned char)r->imageData[i*r->widthStep + j] >= 220 &&
				(unsigned char)r->imageData[i*r->widthStep + j] <= 255 &&
				(unsigned char)b->imageData[i*b->widthStep + j] >= 220 &&
				(unsigned char)b->imageData[i*b->widthStep + j] <= 255)
			{
				dst->imageData[i*dst->widthStep + j] = (unsigned char)255;
			}
			// Ȳ�� ���� ����. �ش� ������ �������.
			else if ((unsigned char)g->imageData[i*g->widthStep + j] >= 180 &&  //150
				(unsigned char)g->imageData[i*g->widthStep + j] <= 240 && //200
				(unsigned char)r->imageData[i*r->widthStep + j] >= 180 && //170
				(unsigned char)r->imageData[i*r->widthStep + j] <= 255 && //255
				(unsigned char)b->imageData[i*b->widthStep + j] >= 30 &&  //30
				(unsigned char)b->imageData[i*b->widthStep + j] <= 200) //200
			{
				dst->imageData[i*dst->widthStep + j] = (unsigned char)255;
			}
			else // ������ ������ ���������� �����.
			{
				dst->imageData[i*r->widthStep + j] = 0;
			}
		}
	}
}

void cvDetectLine(IplImage* org, IplImage* src, IplImage* dst)
{
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	int i;
	CvPoint *point;

	cvCanny(src, dst, 30, 230, 3); // ���� ���ϱ�.

								   //CV_HOUGH_STANDARD ���
	/*
	lines = cvHoughLines2(dst, storage, CV_HOUGH_STANDARD, 1, CV_PI / 180, 90, 0, 0);

	for (i = 0; i < MIN(lines->total, 100); i++)
	{
		float* line = (float*)cvGetSeqElem(lines, i);
		float rho = line[0];
		float theta = line[1];
		CvPoint pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		cvLine(org, pt1, pt2, CV_RGB(0, 0, 255), 2, 4);
		for (int k = 0; k < 10000; k++)
		{
			int i = k;
			i = k + i;
		}
	}
	*/
	//CV_HOUGH_PROBABILISTIC ���
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 90, 30, 10);
	for (i = 0; i < lines->total; i++) {
	point = (CvPoint *) cvGetSeqElem (lines, i);
	cvLine (org, point[0], point[1], CV_RGB (255, 255, 0), 2, 4, 0);
	}
}


struct sLaneCand {
	int x, y;
	float mag;
	float ori;

	sLaneCand() : x(0), y(0), mag(0.), ori(0.) { }
	sLaneCand(int x_, int y_, float m_, float o_)
		: x(x_), y(y_), mag(m_), ori(o_) { }
};

void LaneCandidate(IplImage *mag, IplImage *ori, IplImage *dst)
{
	// �̹��� ���� �� �ȼ��� ��ġ�� y,x�� �����Ͽ� �ȼ��� ���� ����Ʈ ���� ����Ѵ�.
#define pPixel(img,y,x)	(img->imageData + (y)*img->widthStep + img->nChannels*img->depth/8*(x))

	// orientation�� ������ Lane �ĺ������� �����ϱ� ���� ����ũ �̹����� �����.
	IplImage *mask = cvCreateImage(cvGetSize(mag), IPL_DEPTH_8U, 1);
	cvZero(mask);

	CvPoint pts[3] = { { 0, mag->height },{ mag->width / 2, 0 },{ mag->width, mag->height } };
	cvFillConvexPoly(mask, &pts[0], 3, cvScalar(255));

	// �� ��ĵ ���ο��� Lane �ĺ� ������ ã������ �ӽ÷� �ʿ��� ���� Ȯ��
	vector<sLaneCand> tmp;
	tmp.reserve(mag->width);

	for (int h = 0; h<mag->height; h++) {
		tmp.clear();

		float *m = (float *)pPixel(mag, h, 0);
		float *o = (float *)pPixel(ori, h, 0);
		char  *d = (char  *)pPixel(dst, h, 0);
		char  *k = (char  *)pPixel(mask, h, 0);

		for (int w = 1, nw = mag->width - 1; w<nw; w++) {
			// ����ũ�� üũ�Ǿ� ���� ���� �������� 
			if (!k[w]) {
				// orientation�� ������ ����� ������ Lane �ĺ��� �� �� ����.
				// ����: orientation�� lane�� edge�� ������ ������ ǥ���Ѵ�.
				const double margin = 0.5;

				if ((M_PI * 2 - margin < o[w] || o[w] < +margin) ||
					(M_PI - margin < o[w] && o[w] < M_PI + margin)) continue;
			}

			// magnitude�� threshold�� �Ѿ�鼭 �ִ밡 �Ǵ� ���� ã�´�.
			const double threshold = 0.15;

			if (threshold < m[w] && m[w - 1] < m[w] && m[w] > m[w + 1]) {
				// For Debugging: �帮�� �̹��� �� ǥ���Ѵ�.
				// d[w] = (m[w] <= 1.) ? m[w]*255 : 255;

				// �� ���� �ϴ� tmp�� ������ �״ٰ� �ٽ� Lane �ĺ��� �Ǳ� ���� �ٸ� ���ǵ��� �߰��� ���캻��.
				tmp.push_back(sLaneCand(w, h, m[w], o[w]));
			}
		}

		// h ��ġ���� Lane�� ���� ����:
		// h == 0 �� �� lane�� ���� 0
		// h == height�� �� lane�� ���� 50 pixel
		// Lane ���� +-25% ������ üũ�Ѵ�.
		const double lane_width_max = 50.;
		double lane_width = h*lane_width_max / mag->height;
		int lane_width_lo = -3 + (int)(lane_width*0.8);
		int lane_width_hi = 3 + (int)(lane_width*1.2);

		// h ��ġ���� Lane�� ���� �� �ִ� �ִ� �� ���� ���� ������ ���Ͽ�
		// Lane�� �ܰ����� �Ǵ� �������� �˻��Ѵ�.
		// �ܰ��� ���� 1: ���� ���(magnitude)�� ����ؾ� �Ѵ�.
		//        ���� 2: ���� ����(orientation)�� 180�� ���̰� �����Ѵ�.
		const float mag_threshold = 0.2f;
		const float ori_threshold = 0.5f;

		for (int i = 0; i<(int)tmp.size() - 1; i++) {
			if (M_PI / 2 < tmp[i].ori && tmp[i].ori < M_PI * 3 / 2) {
				continue;
			}

			for (int j = i + 1; j<(int)tmp.size(); j++) {
				int width = tmp[j].x - tmp[i].x;

				if (lane_width_lo <= width && width <= lane_width_hi) {
					float mag_err = (tmp[j].mag - tmp[i].mag) / (tmp[j].mag + tmp[i].mag);
					float ori_err = DeltaRad(DeltaRad(tmp[j].ori, tmp[i].ori), M_PIf);

					if ((-mag_threshold < mag_err && mag_err < +mag_threshold) &&
						(-ori_threshold < ori_err && ori_err < +ori_threshold)) {

						int x_mid = (tmp[j].x + tmp[i].x) / 2;
						float mag_avg = (tmp[j].mag + tmp[i].mag) / 2.f;
						// ����� �� �� edge�� ������ ���� pi/2�� ���� edge�� �����ϰ� �ٲ۴�.
						float ori_avg = (tmp[j].ori + tmp[i].ori) / 2.f + M_PIf;

						d[x_mid] = (mag_avg <= 1.) ? (int)(mag_avg * 255) : 255;

						// For Debugging: �̹��� �� Lane �ĺ����� ǥ���Ѵ�.
						cvCircle(draw, cvPoint(x_mid, h), 2, CV_RGB(0, 0, 255));
					}
				}
				else if (width > lane_width_hi) {
					break;
				}
			}
		}
	}

	cvReleaseImage(&mask);
}

sPoint VanishingPoint(vector<sLine> &cand, vector<sLine> *inlier)
{
	const double distance_threshold = 10.;

	ransac_vanishing_point(cand, _vp, 20);

	if (inlier) {
		inlier->reserve(cand.size());

		for (unsigned int i = 0; i<cand.size(); i++) {
			double d = compute_distance(cand[i], _vp);
			if (d < distance_threshold) {
				inlier->push_back(cand[i]);
			}
		}
	}

	return _vp;
}

double ransac_vanishing_point(vector<sLine> &data, sPoint &model, double distance_threshold)
{
	const int no_samples = 2;

	if (data.size() < no_samples) {
		return 0.;
	}

	vector<sLine> samples;

	vector<sLine> inliers;
	inliers.reserve(data.size());

	sPoint estimated_model;
	double max_cost = 0.;

	int max_iteration = (int)(1 + log(1. - 0.99) / log(1. - pow(0.5, no_samples)));

	for (int i = 0; i<max_iteration; i++) {
		if (i == 0) {
			// ó�� �� ���� ���� ������ �𵨷� �˰����� �����Ѵ�.
			estimated_model = model;
		}
		else {
			// 1. hypothesis

			// ���� �����Ϳ��� ���Ƿ� N���� ���� �����͸� ����.
			samples = get_samples(no_samples, data);

			// �� �����͸� �������� �����ͷ� ���� �� �Ķ���͸� �����Ѵ�.
			//estimated_model = compute_model_parameter(samples);
		}

		// 2. Verification

		// ���� �����Ͱ� ������ �𵨿� �� �´��� �˻��Ѵ�.
		inliers.clear();
		double cost = model_verification(inliers, estimated_model, data, distance_threshold);

		// ���� ������ ���� �� �´´ٸ�, �� �𵨿� ���� ��ȿ�� �����ͷ� ���ο� ���� ���Ѵ�.
		if (max_cost < cost) {
			max_cost = cost;

			//model = compute_model_parameter(inliers);
		}
	}

	return max_cost;
}

bool find_in_samples(const vector<sLine> &samples, const sLine &data)
{
	for (unsigned int i = 0; i<samples.size(); ++i) {
		if (samples[i].sx == data.sx && samples[i].sy == data.sy &&
			samples[i].ex == data.ex && samples[i].ey == data.ey) {
			return true;
		}
	}
	return false;
}

vector<sLine> get_samples(int no_samples, const vector<sLine> &data)
{
	vector<sLine> samples;
	samples.reserve(no_samples);

	// �����Ϳ��� �ߺ����� �ʰ� N���� ������ ������ ä���Ѵ�.
	do {
		int index = rand() % data.size();

		if (!find_in_samples(samples, data[index])) {
			samples.push_back(data[index]);
		}
	} while ((int)samples.size() < no_samples);

	return samples;
}



double compute_distance(const sLine &line, const sPoint &x)
{
	// �� ��(x)�κ��� ����(line)�� ���� ������ ����(distance)�� ����Ѵ�.
	return fabs((double)((x.x - line.sx)*line.dy - (x.y - line.sy)*line.dx)) / line.len;
}

double model_verification(vector<sLine> &inliers, const sPoint &model, const vector<sLine> &data, double distance_threshold)
{
	double cost = 0.;

	for (unsigned int i = 0; i<data.size(); i++) {
		// ������ ���� ������ ���̸� ����Ѵ�.
		double distance = compute_distance(data[i], model);

		// ������ �𵨿��� ��ȿ�� �������� ���, ��ȿ�� ������ ���տ� ���Ѵ�.
		if (distance < distance_threshold) {
			cost += data[i].len;

			inliers.push_back(data[i]);
		}
	}

	return cost;
}

inline double DeltaRad(double ang1, double ang2)
{
	double da = ang1 - ang2;
	if (-M_PI < da && da < M_PI) return da;
	else {
		da = fmod(da, 2 * M_PI);
		if (M_PI <= da) return da - 2 * M_PI;
		else if (da <= -M_PI) return da + 2 * M_PI;
		else return da;
	}
	return da;
}

inline int INT_(const double a)
{
	// return (long)floor (a + 0.5);
	return (0 < a) ? (int)(a + 0.5) : (int)(a - 0.5);
}


void Draw_Line(Mat* input_Matrix_1, Mat* input_Matrix_2, vector<Vec2f> input_vector)
{
	vector<Vec2f>::const_iterator it = input_vector.begin();

	int left_line_count = 1;
	int right_line_count = 1;
	int pt1_left_y_value = 0;
	int pt2_left_y_value = 0;
	int pt1_right_y_value = 0;
	int pt2_right_y_value = 0;
	int Left_lane_x = 0;
	int Right_lane_x = 0;
	float Left_Distance = 0;
	float Right_Distance = 0;
	float Left_ratio = 0;
	float Right_ratio = 0;
	char Left_ratio_string[8];
	char Right_ratio_string[8];
	char Frame_sec[16];
	char Real_time_speed[16];

	Point P_ratio((int)((input_Matrix_1->cols) / 2), 0);

	while (it != input_vector.end()) {
		float rho = (*it)[0];
		float theta = (*it)[1];

		if (theta < CV_PI / 4. || theta > 3.*CV_PI / 4.) {
			cv::Point pt1((int)(rho / cos(theta)), 0);
			cv::Point pt2((int)((rho - input_Matrix_1->rows*sin(theta)) / cos(theta)), (int)(input_Matrix_1->rows));

			//			cv::line(*input_Matrix_1, pt1, pt2, cv::Scalar(0, 0, 255), 2);
			//			cv::line(*input_Matrix_2, pt1, pt2, cv::Scalar(0, 0, 255), 2);

			if ((pt1.y < -ANGLE_THRESHOLD && pt2.y > ANGLE_THRESHOLD) || (pt1.y > ANGLE_THRESHOLD && pt2.y < -ANGLE_THRESHOLD)) {
				//				cv::line(*input_Matrix_1, pt1, pt2, cv::Scalar(0, 0, 255), 5);
				//				cv::line(*input_Matrix_2, pt1, pt2, cv::Scalar(0, 0, 255), 5);

				if (pt1.y < -ANGLE_THRESHOLD && pt2.y > ANGLE_THRESHOLD) {
					pt1_right_y_value += pt1.y;
					pt2_right_y_value += pt2.y;
					right_line_count++;
				}
				else {
					pt1_left_y_value += pt1.y;
					pt2_left_y_value += pt2.y;
					left_line_count++;
				}

				fout << "pt1.x = " << pt1.x << "\tpt1.y = " << pt1.y << "  �ڡڡڡڡڡڡ�if" << endl;
				fout << "pt2.x = " << pt2.x << "\tpt2.y = " << pt2.y << "  �ڡڡڡڡڡڡ�if" << endl;
			}
			else {
				fout << "pt1.x = " << pt1.x << "\tpt1.y = " << pt1.y << endl;
				fout << "pt2.x = " << pt2.x << "\tpt2.y = " << pt2.y << endl;
			}

		}
		else {
			cv::Point pt1(0, (int)(rho / sin(theta)));
			cv::Point pt2((int)input_Matrix_1->cols, (int)((rho - input_Matrix_1->cols*cos(theta)) / sin(theta)));

			//			cv::line(*input_Matrix_1, pt1, pt2, cv::Scalar(0, 0, 255), 2);
			//			cv::line(*input_Matrix_2, pt1, pt2, cv::Scalar(0, 0, 255), 2);

			if ((pt1.y < -ANGLE_THRESHOLD && pt2.y > ANGLE_THRESHOLD) || (pt1.y > ANGLE_THRESHOLD && pt2.y < -ANGLE_THRESHOLD)) {
				//				cv::line(*input_Matrix_1, pt1, pt2, cv::Scalar(0, 0, 255), 5);
				//				cv::line(*input_Matrix_2, pt1, pt2, cv::Scalar(0, 0, 255), 5);

				if (pt1.y < -ANGLE_THRESHOLD && pt2.y > ANGLE_THRESHOLD) {
					pt1_right_y_value += pt1.y;
					pt2_right_y_value += pt2.y;
					right_line_count++;
				}
				else {
					pt1_left_y_value += pt1.y;
					pt2_left_y_value += pt2.y;
					left_line_count++;
				}

				fout << "pt1.x = " << pt1.x << "\tpt1.y = " << pt1.y << "  �ڡڡڡڡڡڡ�else" << endl;
				fout << "pt2.x = " << pt2.x << "\tpt2.y = " << pt2.y << "  �ڡڡڡڡڡڡ�else" << endl;
			}
			else {
				fout << "pt1.x = " << pt1.x << "\tpt1.y = " << pt1.y << endl;
				fout << "pt2.x = " << pt2.x << "\tpt2.y = " << pt2.y << endl;
			}
		}
		it++;
	}

	/* ���������� ����� �������� ����� ���Ѵ� (����ȭ) */
	pt1_right_y_value = pt1_right_y_value / right_line_count;
	pt2_right_y_value = pt2_right_y_value / right_line_count;

	/* �������� ����� �������� ����� ���Ѵ� (����ȭ) */
	pt1_left_y_value = pt1_left_y_value / left_line_count;
	pt2_left_y_value = pt2_left_y_value / left_line_count;

	/* ��ճ� Point_1 ( ������������ ���� ���� point ) */
	Point average_pt1(0, pt1_right_y_value);
	/* ��ճ� Point_2 ( ������������ ������ �� point ) */
	Point average_pt2((int)input_Matrix_1->cols, pt2_right_y_value);

	cvLine(input_Matrix_1, average_pt1, average_pt2, cv::Scalar(0, 0, 255), 3);		// Edge image�� ������ ������ �׾��ش�
	cvLine(input_Matrix_2, average_pt1, average_pt2, cv::Scalar(0, 0, 255), 3);		// Final image�� ������ ������ �׾��ش�

	average_pt1.y = pt1_left_y_value;	/* ��ճ� Point_1 ( ���������� ���� ���� point ) */
	average_pt2.y = pt2_left_y_value;	/* ��ճ� Point_2 ( ���������� ������ �� point ) */

	cvLine(input_Matrix_1, average_pt1, average_pt2, cv::Scalar(0, 0, 255), 3);		// Edge image�� ������ ������ �׾��ش�
	cvLine(input_Matrix_2, average_pt1, average_pt2, cv::Scalar(0, 0, 255), 3);		// Final image�� ������ ������ �׾��ش�

																					/* ����� ���μ��� �׾��ֱ����� point 2���� ������ �ش� */
	average_pt1.x = (int)((input_Matrix_1->cols) / 2);
	average_pt1.y = 0;
	average_pt2.x = (int)((input_Matrix_1->cols) / 2);
	average_pt2.y = (int)input_Matrix_1->rows;

	cvLine(input_Matrix_2, average_pt1, average_pt2, cv::Scalar(255, 0, 0), 3);		// Final image�� ���߾� ���μ��� �׾��ش�.


	cout << "output<< ROI height = " << CROPPING_HEIGHT << endl;
	cout << "output<< P_ratio.x = " << P_ratio.x << endl;
	cout << "output<< P_ratio.y = " << CROPPING_HEIGHT - P_ratio.y << endl;

	/* ���� �������� �Ÿ��� ã�´� */
	Left_lane_x = ((P_ratio.y - pt1_left_y_value)*((int)input_Matrix_1->cols)) / (pt2_left_y_value - pt1_left_y_value);	// �� ���� ������ ������ ���������� ���� y���� �־� x��ǥ�� ã�´�.
	cout << "output<< Left lane x value = " << Left_lane_x << endl;
	Left_Distance = P_ratio.x - Left_lane_x;		// ���������� P �������� �Ÿ��� ���Ѵ�.
	cout << "output<< Left lane distance = " << Left_Distance << endl;


	/* ������ �������� �Ÿ��� ã�´� */
	Right_lane_x = ((P_ratio.y - pt1_right_y_value)*((int)input_Matrix_1->cols)) / (pt2_right_y_value - pt1_right_y_value); // �� ���� ������ ������ ���������� ���� y���� �־� x��ǥ�� ã�´�.
	cout << "output<< Right lane x value = " << Right_lane_x << endl;
	Right_Distance = Right_lane_x - P_ratio.x;		// ������������ P �������� �Ÿ��� ���Ѵ�.
	cout << "output<< Right lane distance = " << Right_Distance << endl << endl;

	Point Left_intersection(Left_lane_x, P_ratio.y);		// P ���̿����� ������������ ����
	Point Right_intersection(Right_lane_x, P_ratio.y);		// P ���̿����� �������������� ����

															/* ���������� P���� ������ �׸��� */
	cvLine(input_Matrix_2, Left_intersection, P_ratio, cv::Scalar(0, 255, 255), 3);
	/* ������������ P���� ������ �׸��� */
	cvLine(input_Matrix_2, P_ratio, Right_intersection, cv::Scalar(0, 255, 255), 3);

	/* �¿������ �������� P�� ȭ�鿡 �׸��� */
	cvCircle(input_Matrix_2, P_ratio, 5, Scalar(0, 0, 255), 2);
	cvCircle(input_Matrix_2, P_ratio, 2, Scalar(0, 0, 255), 2);

	/* ������ ã�´� */
	//	if (Left_Distance > Right_Distance)	{
	Left_ratio = Left_Distance / Right_Distance;
	Right_ratio = 1;
	//	}
	//	else   {
	//		Left_ratio = 1;
	//		Right_ratio = Right_Distance / Left_Distance;
	//	}

	cout.precision(2);
	cout << fixed << "output<< Ratio = " << Left_ratio << " : " << Right_ratio << endl;

	printf(Left_ratio_string, "%.2f", Left_ratio);
	printf(Right_ratio_string, "%.2f", Right_ratio);

	/* ȭ�� �¿쿡 ������ ������ �׷��ش� */
	//cvPutText(input_Matrix_2, Left_ratio_string, Point(10, ((int)input_Matrix_1->rows / 2) + 30), 3, 1.5,Scalar(0, 255, 255), 6);
	//cvPutText(input_Matrix_2, Right_ratio_string, Point((int)input_Matrix_1->cols - 115, ((int)input_Matrix_1->rows / 2) + 30), 3, 1.5, Scalar(0, 255, 255), 6);


	/* ȭ�� ������ܿ� Frame/Sec �� Real time Speed�� �׷��ش� */
	//	putText(*input_Matrix_2, Frame_sec, Point(10, 40), 3, 1, Scalar(0, 255, 255), 3);
	//	putText(*input_Matrix_2, Real_time_speed, Point(10, 80), 3, 1, Scalar(0, 255, 255), 3);
}

double getRotationAngle(const IplImage* src)
{
	// Only 1-Channel
	if (src->nChannels != 1)
		return 0;

	// ������ �� ����� �� �ֵ��� ��â
	cvDilate((IplImage*)src, (IplImage*)src);

	// ���念�� ����
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* seqLines;

	// Image�� ���������� seq�� ����(rho, theta)
	seqLines = cvHoughLines2((IplImage*)src, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 50, 30, 3); // ���ڵ� �����ؾ���

																										  // �������� �Ÿ��� ���ؼ� ���� �� ������ �������� �̹��� 0 or 90���� ȸ��
	double    longDistance = 0;    // ���� �� ���� �� ����
	int        longDistanceIndex = 0;    // ���� �� ���� �� ���� �ε���
	for (int i = 0; i < seqLines->total; i++) {
		CvPoint* lines = (CvPoint*)cvGetSeqElem(seqLines, i);
		double euclideanDistance;        // sequence�� ����� line���� Euclidean distance�� ����
		euclideanDistance = (lines[1].x - lines[0].x) * (lines[1].x - lines[0].x) + (lines[1].y - lines[0].y) * (lines[1].y - lines[0].y);
		euclideanDistance = sqrt(euclideanDistance);

		// ���� �� Euclidean distance�� ���� 
		if (longDistance < euclideanDistance) {
			longDistanceIndex = i;
			longDistance = euclideanDistance;
		}

		//// �������� ����
		//cvLine((IplImage*)src, lines[0], lines[1], CV_RGB(0, 0, 0), 3, CV_AA);
	}
	//// check the longest line
	//CvPoint* lines = (CvPoint*)cvGetSeqElem(seqLines, longDistanceIndex);
	//cvLine((IplImage*)src, lines[1], lines[0], CV_RGB(125, 125, 125), 3, CV_AA);
	//cout<<"pt1("<<lines[0].x<<", "<<lines[0].y<<")   pt2("<<lines[1].x<<", "<<lines[1].y<<")"<<endl;

	// ȸ���� ���� ���
	CvPoint*    lines = (CvPoint*)cvGetSeqElem(seqLines, longDistanceIndex);
	int            dx = lines[1].x - lines[0].x;
	int            dy = lines[1].y - lines[0].y;
	double        rad = atan2((double)dx, (double)dy);    // ȸ���� ����(radian)
	double        degree = abs(rad * 180) / CV_PI;            // ȸ���� ����(degree) ����

															  // ȸ���� �������� 
	if (degree>90)
		degree -= 90;

	// �޸� ����
	cvClearSeq(seqLines);
	cvReleaseMemStorage(&storage);

	return degree;
}


