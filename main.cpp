#include "myHead.hpp"
#include <opencv.hpp>
#include "fstream"
#include "opticalFlow.h"
#include "getConfig.h"
#include <sstream>

using namespace cv;
using namespace std;

#define IMAGE_WIDTH 640    	//图像宽度
#define IMAGE_HEIGHT 480   	//图像高度
#define ZOOM_FACTOR 1      	//为加快处理速度，对图像进行等比例缩小，此处为原比例
#define DEBUG            	//图像绘制与显示，在调试时取消注释，在设为开机自动运行时必须注释掉
#define BLUE				//目标的颜色，或者为RED
#define CAM					//无用
#define DARK				//周围环境是暗还是亮，目前测试还是DARK效果比较好
#define SEND_NUM			//发送格子个序号，与SEND_ANGLE互斥
//#define SEND_ANGLE		//发送yaw和pitch两个角度
#define DRONE_DETECT		//打开第二个摄像头检测无人机，注释掉或只有一个摄像头则不检测
#define minContour 15		//最小轮廓尺寸，用于控制检测目标的远近
#define maxContour 600		//最大轮廓尺寸，用于滤掉过大的错误目标

bool stopProc = false;		//退出线程标志
bool undistortMode = true;	//畸变矫正标志，无用
//bool imgCatched = false;
bool thresholdMode = false;	//二值图显示标志
bool paused = false;		//暂停标志

int sendPeriod = 20; //发送周期，ms
const char *g_szTitle = "Camera";//窗口名称
map<string,string> config; //从video.cfg读取的参数

//初始化
#ifdef SEND_ANGLE
double anglePitch = 0, angleYaw = 0;
#else
char regionNum[1] = { 0xFF };
#endif

//发送线程
void *sendData(void *)
{
#ifndef WIN32
    Serialport stm32("/dev/ttyS0");
#ifdef SEND_ANGLE
    while (!stopProc)
    {
        //cout<<angleYaw<<"    "<<anglePitch<<endl;
        stm32.sendAngle(angleYaw, anglePitch);
        usleep(sendPeriod * 1000);
    }
#else
    while(!stopProc)
    {
        if(!paused)
        {
            stm32.send(regionNum);
            cout << hex << (int)regionNum[0] << endl;
        }
        usleep(sendPeriod * 1000);
    }
#endif // SEND_ANGLE
#endif // LINUX
    cout<<"send data pthread exit\n"<<endl;
}

//检测无人机线程
void *droneDetect(void *)
{
#ifdef DRONE_DETECT
    VideoCapture cap(1);
    while (!cap.isOpened()&&!stopProc)
    {
        cout << "can't open the drone detection camera!\n";
        //cout << "drone detection pthread exit\n" << endl;
        cap.open(1);
        usleep(1000 * 1000);
    }
    Mat src,result;
    OpticalFlow droneOF(50,0.01,10);
    int numSend = 0;
    while(!stopProc)
    {
        int64 t0 = getTickCount();
        cap >> src;
        if (src.empty())
        {
            cerr << " --(!) No captured frame -- Break!\n";
            break;
        }

        droneOF.tracking(src);
        result = droneOF.output;
// 			Point2f vertex[4];
// 			droneOF.motionObject.points(vertex);
// 			for (int i = 0; i < 4; ++i)
// 				line(result, vertex[i], vertex[(i + 1) % 4], Scalar(0, 0, 200),2);

        if (droneOF.isDroneAppear())
        {
#ifdef DEBUG
            vector<Point2f> points = droneOF.movingPoints;
            vector<int> hull;
            convexHull(Mat(points), hull, true);
            int hullcount = (int)hull.size();//Í¹°üµÄ±ßÊý
            Point point0 = points[hull[hullcount - 1]];//Á¬œÓÍ¹°ü±ßµÄ×ø±êµã
            for (int i = 0; i < hullcount; i++)
            {
                Point point = points[hull[i]];
                line(result, point0, point, Scalar(255, 0, 0), 2, CV_AA);
                point0 = point;
            }
#endif // DEBUG
            numSend++;
            if(numSend >= 2)
            {
                cout << "target emerged\n";
                regionNum[0]=0xee;
            }
        }
        else
        {
            numSend = 0;
            //cout << "no drones\n";
        }
        int key = waitKey(1);
#ifdef DEBUG
        //int64 t = getTickCount() - t0;
        //cout << t * 1000 / getTickFrequency() << endl;
        //imshow("optical flow", result);
        if ((char)key == 27)
            break;
#endif // DEBUG

    }
    cap.~VideoCapture();
#endif // DRONE_DETECT
    regionNum[0] = 0xf1;
    cout<<"drone detection pthread exit\n"<<endl;
}

//主线程，检测装甲
int main()
{
    ReadConfig("video.cfg",config);//读取配置信息
    int t = atoi(config["t"].c_str());//得到阈值参数t
    VideoCapture cap(0);
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);
    if (!cap.isOpened())
    {
        cerr << "can't open the armor detection camera!\n";
        return 0;
    }

	//生成两个子线程（在linuux下）
#ifndef WIN32
    pthread_t id,id2;
    int ret = pthread_create(&id, NULL, sendData, NULL);
    int ret2 = pthread_create(&id2, NULL, droneDetect, NULL);
#endif //创建线程

#ifdef DEBUG
    namedWindow(g_szTitle);
    createTrackbar("t", g_szTitle, &t, 256, 0);
#else//在没有定义DEBUG时，生成记录视频
    string videoName;
    videoName += config["num"];
    videoName += ".avi";
    VideoWriter writer(videoName, CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(640, 480));
    config["num"] = to_string(atoi(config["num"].c_str()) + 1);
    WriteConfig("video.cfg",config);
#endif // DEBUG

    Mat src, imgThresholded;
    cap >> src;

    Mat imgOriginal = src(Rect(0,(int)IMAGE_HEIGHT/3,IMAGE_WIDTH,(int)IMAGE_HEIGHT*2/3));//由于图像上半部分无有效信息，故取src图像的下面2/3
    Rect roiImg = Rect(0, 0, imgOriginal.cols * ZOOM_FACTOR, imgOriginal.rows * ZOOM_FACTOR);

    vector<RotatedRect> vEllipse;//符合条件的椭圆
    Armors armors(imgOriginal.cols*ZOOM_FACTOR, imgOriginal.rows*ZOOM_FACTOR);//实例化Armors类
    vector<vector<Point> > contours;//检测到的轮廓

    int sendFilter = 0;
    bool sendBool = false;
    while (true)
    {
        String noTargetReason;//检测不到目标输出的信息
        int64 t0 = getTickCount();
        if (!paused)
        {
            if (!cap.read(src))
                break;
#ifndef DEBUG
            writer << src;
#endif // DEBUG
            imgOriginal = src(Rect(0,(int)IMAGE_HEIGHT/3,IMAGE_WIDTH,(int)IMAGE_HEIGHT*2/3));
            resize(imgOriginal, imgOriginal, Size(imgOriginal.cols * ZOOM_FACTOR, imgOriginal.rows * ZOOM_FACTOR));//等比例缩小
            //imgOriginal -= Scalar(B,G,R);//亮度调整
#ifndef DARK
#ifdef BLUE
            GetDiffImage(imgOriginal, imgThresholded, t, blue, roiImg);//蓝色
#else
            GetDiffImage(imgOriginal, imgThresholded,t , red, roiImg);//红色
#endif
#else//Dark
            GetBrightImage(imgOriginal, imgThresholded, t, roiImg);
#endif // DARK

            preProcess(imgThresholded);//预处理
            findContours(imgThresholded, contours, RETR_CCOMP, CHAIN_APPROX_NONE);

#ifdef DEBUG
            drawContours(imgThresholded, contours, -1, Scalar(255, 255, 255));
#endif // DEBUG

            for (auto itContours = contours.begin(); itContours != contours.end(); ++itContours)
            {
                vector<Point> points = *itContours;//将一个轮廓的所有点存入points
                if (itContours->size() > minContour && itContours->size() < maxContour)//筛选轮廓上的点大于100的轮廓
                {
                    RotatedRect s = fitEllipse(Mat(points));//拟合椭圆

                    if ((s.size.height < s.size.width) || (s.size.height * s.size.width > 2000))//|| (s.size.height / s.size.width > 12))
                    {
                        //cout<<s.size<<endl;
                        noTargetReason += "size don't cater to standard.\n";
                        continue;
                    }
#ifdef DARK
                    Rect colorRect = (s.boundingRect() - Point(7, 7) + Size(14, 14)) & roiImg;
                    int color = colorJudge(imgOriginal(colorRect));
#ifdef BLUE
                    if (color == blue)
                        vEllipse.push_back(s);
                    else
                        continue;
#endif // BLUE
#ifdef RED
                    if (color == red || color == purple)
                        vEllipse.push_back(s);
                    else
                        continue;

#endif // RED
#else//Bright
                    vEllipse.push_back(s);
#endif // DARK
#ifdef DEBUG
                    ellipse(imgOriginal, s, Scalar(255,255, 66),2);
#endif // DEBUG
                }
                else
                {
                    noTargetReason += "size of contours is too small or too big.\n";
                }

                points.clear();//points.swap(vector<Point>());
            }
			
            armors.inputEllipse(vEllipse);//输入将测到的椭圆，寻找装甲
            Point2f target = armors.getTarget();//求目标坐标
            //target = armors.track();//追踪

            if (armors.number() > 0)//目标没有丢失
            {
                roiImg = armors.getROIbox(imgOriginal);
#ifdef DEBUG
                armors.drawAllArmors(imgOriginal);
                circle(imgOriginal, target*ZOOM_FACTOR, 5, Scalar(0, 255, 255), 1, 8, 3);
                line(imgOriginal, target, target + armors.getVelocity(), Scalar(240, 33, 22), 2, CV_AA);
#endif//DEBUG

#ifdef SEND_ANGLE
                target *= 1 / ZOOM_FACTOR;//恢复到原图的实际像素
                Point3f xyz = img2camera(target, intrinsic_matrix);
                angleYaw = atan(xyz.x / xyz.z) * 180 / CV_PI;
                anglePitch = atan(xyz.y / sqrt(xyz.z*xyz.z + xyz.x*xyz.x)) * 180 / CV_PI;
#ifdef DEBUG
                char yawText[32] = "Yaw:  ", pitchText[32] = "Pitch: ";
                char yaw[32], pitch[32];
#ifndef WIN32
                sprintf(yaw, "%f", angleYaw);
                strcat(yawText, yaw);
                sprintf(pitch, "%f", anglePitch);
                strcat(pitchText, pitch);
#else
                sprintf_s(yaw, "%f", angleYaw);
                strcat_s(yawText, yaw);
                sprintf_s(pitch, "%f", anglePitch);
                strcat_s(pitchText, pitch);
#endif // LINUX
                putText(imgOriginal, yawText, Point(10, 30), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2, 8);
                putText(imgOriginal, pitchText, Point(10, 60), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2, 8);
#endif //DEBUG
#else

                if(sendFilter>=2)
                    sendBool = true;
                else
                    sendFilter++;

                if(sendBool)
                    regionNum[0] = armors.getTargetRegion();
                else
                    regionNum[0] = 0xff;
#endif //发送角度模式

            }
            else//目标丢失
            {
#ifdef DEBUG
                //noTargetReason = armors.error;
                cout << noTargetReason << endl;
                putText(imgOriginal, "Looking for the enemy.......", Point(90*ZOOM_FACTOR, 90*ZOOM_FACTOR), FONT_HERSHEY_PLAIN, 2*ZOOM_FACTOR, Scalar(0, 0, 255), 2, 8);
#endif // DEBUG

                roiImg = Rect(0, 0, imgOriginal.cols * ZOOM_FACTOR, imgOriginal.rows * ZOOM_FACTOR);
#ifdef SEND_ANGLE
                anglePitch = 0;
                angleYaw = 0;
#else
                sendFilter--;
                if(sendFilter < 0)
                {
                    sendFilter = 0;
                    sendBool = false;
                    regionNum[0] = 0xff;
                }

#endif // SEND_ANGLE
            }
            vEllipse.clear();
            contours.clear();//contours.swap(vector<vector<Point>>());
        }
        char key = (char)waitKey(1);

#ifdef DEBUG
        if (key == 27)  break;  //esc键退出
#ifdef SEND_NUM
        drawGrid(imgOriginal);
#endif // SEND_NUM
        if (thresholdMode)
            imshow(g_szTitle, imgThresholded);
        else
            imshow(g_szTitle, imgOriginal);
        switch (key)
        {
        case 'b':
            thresholdMode = !thresholdMode;
            break;
        case 'p':
            paused = !paused;
            break;
        case 'd':
            undistortMode = !undistortMode;
            break;
        default:
            ;
        }
        int64 t = getTickCount() - t0;
        //cout << "主线程：" <<1000 * t/getTickFrequency() << "ms" << "\n";
#endif // DEBUG
    }

#ifndef WIN32
    stopProc = true;
    pthread_join(id, NULL);//wait the end of id
    pthread_join(id2, NULL);//wait the end of id2
#endif // LINUX
    cap.~VideoCapture();
    destroyAllWindows();
    config["t"]=to_string(t);
    WriteConfig("video.cfg",config);
    cout<<"main pthread exit\n"<<endl;
    regionNum[0] = 0xf0;
    return 0;
}
