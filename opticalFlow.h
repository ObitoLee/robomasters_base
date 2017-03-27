#include <opencv.hpp>
#include <iostream>
#include <cstdio>

using namespace std;
using namespace cv;

/************************************************************************
@brief：通过金字塔Lucas - Kanade 光流方法计算某些点集的光流（稀疏光流）
************************************************************************/
class OpticalFlow
{
public:
	OpticalFlow(int _max_corners, double _quality_level, double _min_dist);
	OpticalFlow();
	~OpticalFlow();
	void tracking(Mat &_frame);
	Mat output;
	vector<Point2f> points[2];	// point0为特征点的原来位置（被跟踪的点），point1为当前帧的特征点，由光流法计算出来
	vector<Point2f> initial;	// 跟踪点的初始位置
	vector<Point2f> displacement;
	int getPointsNum(){ return pointsNum; };
	bool isDroneAppear();
	RotatedRect motionObject;
	vector<Point2f> movingPoints;
private:
	bool addNewPoints();
	bool acceptTrackedPoint(int _i);
	Mat gray;					// 当前帧
	Mat gray_prev;				// 前一帧
	vector<Point2f> features;	// 检测的特征点
	int max_corners;			// 检测的最大特征数
	double quality_level;		// 特征检测的等级
	double min_dist;			// 两特征点之间的最小距离
	vector<uchar> status;		// 跟踪特征的状态，特征的流发现为1，否则为0
	vector<float> err;			// 新旧两个特征点位置的误差
	int pointsNum;				//被追踪特征点的数量
};


