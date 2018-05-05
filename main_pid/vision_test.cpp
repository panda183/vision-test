#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "OpenNIHelper.h"
#include <unistd.h>
#include <chrono>
#include <stdlib.h>
#include "Hal.h"
#include "LCDI2C.h"
#include "api_i2c_pwm.h"

using namespace cv;
using namespace std;
using namespace EmbeddedFramework;

#ifndef DEBUG_
    #define DEBUG_
#endif

#define SW1_PIN	160
#define SW2_PIN	161
#define SW3_PIN	163
#define SW4_PIN	164
#define SENSOR	165
#define LED	166

int mode=0;
int minH=100,minS=130,minV= 100,maxH=135,maxS=255,maxV=255;
int xLeftLane=100,xRightLane=210;
int throttle_val = 30;
double theta = 80;
Mat leftSign;
Mat rightSign;
GPIO *gpio;
PCA9685 *pca9685;
I2C *i2c_device;
LCDI2C *lcd;
Mat depthImg, colorImg, grayImage, disparity,birdViewImg,laneImg;
bool sw1_stat,sw2_stat,sw3_stat,sw4_stat;
bool bamlephai=true;
VideoWriter writer,writer2;
int videoidx=0;
float _p=0,_d=0;
int rangeCheck=50;
int signCount=0;
bool vatCan=false;
int speed=35,targetSpeed=55;

void init();
void putTextLCD(int x, int y,char *msg);
void clearLCD();
bool nothingFrontSensor();
bool isPressed(int PIN);
int getLCDkey();
void Menu();
void RunCar();
void setControl(int speed,double angle);
void Ready();
void DetectSign(Mat &src);
int recognizeSign(Mat &sign);
double similar(Mat &img1, Mat &img2);
void equalizeHistBGR(Mat &src, Mat &dst);
double distance(Point p1, Point p2);
Mat birdView(Mat &input);
double getAngleSteering();
double calAngle(Point &target);
void laneMarkingDetector(Mat &srcColor, Mat &dstGray,int tau);
bool colorVatCan(int H){
    if((H>0&&H<10)||(H>40&&H<80)||(H>17&&H<180)) return true;
    return false;
}
void CheckVatCan(){

    int xLaneCheck;
    if(bamlephai){
        xLaneCheck=xRightLane;
    } else {
        xLaneCheck=xLeftLane;
    }

    for(int y=birdViewImg.rows-1;y>240-80;y-=3)
    {
        bool colan=false;
        for(int i=xLaneCheck-rangeCheck/2;i<xLaneCheck+rangeCheck/2;i++){
            if(laneImg.at<uchar>(y,i)==255){
                colan=true;
                xLaneCheck=i;
                break;
            }
        }
        if(!colan){
           for(int i=xLaneCheck-rangeCheck/2;i<xLaneCheck+rangeCheck/2;i++){
                if (birdViewImg.at<Vec3b>(y,i)[1]>125&&birdViewImg.at<Vec3b>(y,i)[2]>125&&colorVatCan(birdViewImg.at<Vec3b>(y,i)[0])){
                    vatCan=true;
                    return;
                }
            }
        }
    }
    //imshow("birdView1",birdViewImg);
    vatCan=false;
}
int main( int argc, char* argv[] )
{
    init();
    Menu();
}
void RunCar()
{
    speed=70;
    //writer.open("./birdViewLane "+to_string(videoidx)+".avi",CV_FOURCC('M','J','P','G'),25.0,Size(320,240),true);
    //writer2.open("./SignDetect "+to_string(videoidx)+".avi",CV_FOURCC('M','J','P','G'),25.0,Size(320,240),true);
    clearLCD();
    putTextLCD(0,0,"Running");

    do{
        int key=getLCDkey();
        if(key==3||!nothingFrontSensor())
        {
            Menu();
        }
        auto cur_time = std::chrono::system_clock::now();
        ni::openni2_getmat(colorImg, depthImg);
        //imshow("color",colorImg);
        laneMarkingDetector(colorImg,laneImg ,12);
        CheckVatCan();
        //imshow("laneImg",laneImg);
        double angle=getAngleSteering();
        DetectSign(colorImg);
        setControl(speed,angle);
        //cout<< chrono::duration<double, milli> (std::chrono::system_clock::now()-cur_time).count()<<endl;
    }
    while( waitKey(1)!=27);
    Menu();
}
void Ready()
{
    clearLCD();
    putTextLCD(0,0,"Ready!!!");
    while(!nothingFrontSensor())
    {
        int key=getLCDkey();
        if(key==3)
        {
            Menu();
        }
    }
    RunCar();
}

void Menu()
{
    xLeftLane=100,xRightLane=210;
    //videoidx++;
    if(writer.isOpened()){
        writer.release();
    }
    if(writer2.isOpened()){
        writer2.release();
    }
    double angle=0;
    setControl(0,0);
    clearLCD();
    putTextLCD(0,0,"Mode:Nghiem Tuc");
    //targetSpeed=55;
    do
    {
        int key=getLCDkey();
        if(key==4)
        {
            mode=(mode+1)%2;
            switch (mode)
            {
            case 0:
                putTextLCD(5,0,"Nghiem Tuc");
                break;
            case 1:
                putTextLCD(5,0,"Thi cu =))");
                break;
            default:
                break;
            }
        }
        if(key==3)
        {
            Ready();
        }
        if(key==2)
        {
            targetSpeed+=5;
        }
        if(key==1)
        {
            targetSpeed-=5;
        }

        string str=to_string(targetSpeed);
        putTextLCD(0,2,(char*)str.c_str());
    }
    while(1);
}
void laneMarkingDetector(Mat &srcColor, Mat &dstGray,int tau)
{
    //xac dinh lan, do rong = tau, anh vao srcGray: anh xam, dstGray: anh tra ve
    Mat srcGray=birdView(srcColor);
    dstGray=srcGray.clone();
    int aux=0;
    for(int j=0; j<srcGray.rows; j++)
    {
        for(int i=tau; i<srcGray.cols-tau; i++)
        {
            aux=srcGray.at<uchar>(j,i)*2;
            aux-=srcGray.at<uchar>(j,i-tau);
            aux-=srcGray.at<uchar>(j,i+tau);
            aux-=abs((int)(srcGray.at<uchar>(j,i-tau)-srcGray.at<uchar>(j,i+tau)));
            aux=(aux<0)?0:aux;
            aux=(aux>255)?255:aux;
            dstGray.at<uchar>(j,i)=aux;
        }
    }
    threshold(dstGray, dstGray, 200, 255, CV_THRESH_BINARY);
}
double getAngleSteering()
{   bool colan=false;
    int xLaneL=-1,xLaneR=-1;
		for(int y=laneImg.rows-1;y>=220;y-=3){
			if(xLaneL==-1&&!bamlephai){
				for(int i=xLeftLane-rangeCheck/2;i<xLeftLane+rangeCheck/2;i++){
					if(i>=0&&i<=laneImg.cols){
						if(laneImg.at<uchar>(y,i)==255){
							xLaneL=i;
                            //line(laneImg,Point(xLaneL-rangeCheck/2,y),Point(xLaneL+rangeCheck/2,y),Scalar(255));
							break;
						}
					}
				}
				xLaneR=xLaneL+70;

			}
			if(xLaneR==-1&&bamlephai){

				for(int i=xRightLane-rangeCheck/2;i<xRightLane+rangeCheck/2;i++){
					if(i>=0&&i<=laneImg.cols){
						if(laneImg.at<uchar>(y,i)==255){
							xLaneR=i;
							break;
						}
					}
				}

				xLaneL=xLaneR-70;
			}
			line(laneImg,Point(xLeftLane-rangeCheck/2,y),Point(xLeftLane+rangeCheck/2,y),Scalar(255));
            line(laneImg,Point(xRightLane-rangeCheck/2,y),Point(xRightLane+rangeCheck/2,y),Scalar(255));

			if(xLaneR!=-1&&xLaneL!=-1){
				xLeftLane=xLaneL;
				xRightLane=xLaneR;
                colan=true;
				break;
			}
		}

		int dau=(bamlephai-0.5)*2;
		int targetPoint=(xLeftLane+xRightLane)/2+dau*8;

        if(vatCan){
            if(colan){
                targetPoint-=20*dau;
            }else{
                targetPoint-=8*dau;
            }
            //cout<<"co vat can"<<endl;

        }
        _d=((160-targetPoint)-_p);
        _p=(160-targetPoint);

		float angle=(_p*4.8+_d);//cout<<_p<<":"<<_d<<":"<<angle<<endl;
		circle(laneImg,Point(targetPoint,239),3,Scalar(255),2);
		//imshow("birdView", laneImg);
		Mat temp;
		cvtColor(laneImg,temp,COLOR_GRAY2BGR);
		//writer.write(temp);
		return angle;

}
double calAngle(Point &target)
{
	// tinh goc tu diem chinh giua anh -> target
    float angle;
    if (target.x != laneImg.cols / 2)
    {
        double k = (target.y * 1.0f - laneImg.rows) / (target.x * 1.0f - laneImg.cols / 2);
        angle = -atan(k) * 180 / 3.14159265;
    }
    else
    {
        angle = 90;
    }
    int dau = angle / abs(angle);
    return (90 - abs(angle)) * dau*-2.8;
}
void DetectSign(Mat &src)
{

    Mat hsv, gray;
    Rect roiDetect = Rect(50, 10, 250, 70);
    cvtColor(src(roiDetect), hsv, COLOR_BGR2HSV);
    Scalar min = Scalar(minH, minS, minV);   //HSV VALUE
    Scalar max = Scalar(maxH, maxS, maxV); //HSV VALUE
    inRange(hsv, min, max, gray);

    erode(gray, gray, Mat(), Point(-1, -1), 2, 1, 1);
    dilate(gray, gray, Mat(), Point(-1, -1), 8, 1, 1);

    vector<vector<Point>> contours;
    findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++)
    {
        Rect r = boundingRect(contours[i]);
        r.x += roiDetect.x;
        r.y += roiDetect.y;
        if(r.width-12>0&&r.height-12>0)
        {
            r.x+=6;
            r.width-=12;
            r.y+=6;
            r.height-=12;
        }


//&&depthImg.at<ushort>(r.y*2+r.height,r.x*2+r.width)>1000&&depthImg.at<ushort>(r.y*2+r.height,r.x*2+r.width)<8000
        if (abs((r.width * 1.0 / r.height) - 1.0) < 0.2&&r.width>10&&r.width<43)
        {//rectangle(src, r, Scalar(0, 0, 255));
            Mat matsign = src(r);
            rectangle(src, r, Scalar(0, 0, 255));
            bool tmp=recognizeSign(matsign);
            if(tmp==bamlephai){
                signCount=0;
            }
            else
            {
                signCount++;
                if(signCount==3){
                    signCount=0;
                    bamlephai=tmp;
                    cout<<"Thay doi"<<endl;
                }
            }
        }
    }
    rectangle(src, roiDetect, Scalar(0, 255, 0));
   // imshow("src", src);
   // writer2.write(src);
}
int recognizeSign(Mat &sign)
{
    double p1 = similar(sign, leftSign);
    double p2 = similar(sign, rightSign);
    if (p1 > p2)
        return 0;
    return 1;
}
Mat birdView(Mat &input)
{
    Mat output;
    // Input Quadilateral or Image plane coordinates
    Point2f inputQuad[4];
    // Output Quadilateral or World plane coordinates
    Point2f outputQuad[4];

    // Lambda Matrix
    Mat lambda;

    // Set the lambda matrix the same type and size as input
    lambda = Mat::zeros(input.rows, input.cols, input.type());
    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input


    inputQuad[0] = Point2f(130, 70);
	inputQuad[1] = Point2f(200, 70);
	inputQuad[2] = Point2f(input.cols, 165);
	inputQuad[3] = Point2f(0, 165);
	// The 4 points where the mapping is to be done , from top-left in clockwise order
	outputQuad[0] = Point2f(100, 0);
	outputQuad[1] = Point2f(210, 0);
	outputQuad[2] = Point2f(210, input.rows-10);
	outputQuad[3] = Point2f(100, input.rows-10);

    // Get the Perspective Transform Matrix i.e. lambda
    lambda = getPerspectiveTransform(inputQuad, outputQuad);
    // Apply the Perspective Transform just found to the src image
    warpPerspective(input, birdViewImg, lambda, output.size());
    //imshow("birdViewImg",birdViewImg);
    cvtColor(birdViewImg, output, COLOR_BGR2GRAY);
    cvtColor(birdViewImg,birdViewImg,COLOR_BGR2HSV);
    //imshow("birdViewHSV",birdViewImg);
    //threshold(output, output, 200, 255, CV_THRESH_BINARY);
    //imshow("thres",output);
    return output;
}
void init()
{
    //=========== Init  =======================================================
    ////////  Init PCA9685 driver   ///////////////////////////////////////////
    pca9685 = new PCA9685() ;
    api_pwm_pca9685_init( pca9685 );
    if (pca9685->error < 0) return;
    //======

    gpio = new GPIO();
    i2c_device = new I2C();
    lcd = new LCDI2C();
    gpio->gpioExport(SW1_PIN);
    gpio->gpioExport(SW2_PIN);
    gpio->gpioExport(SW3_PIN);
    gpio->gpioExport(SW4_PIN);
    gpio->gpioExport(SENSOR);
    gpio->gpioExport(LED);
    gpio->gpioSetDirection(SW1_PIN, INPUT);
    gpio->gpioSetDirection(SW2_PIN, INPUT);
    gpio->gpioSetDirection(SW3_PIN, INPUT);
    gpio->gpioSetDirection(SW4_PIN, INPUT);
    gpio->gpioSetDirection(SENSOR, INPUT);
    gpio->gpioSetDirection(LED, OUTPUT);

    i2c_device->m_i2c_bus = 2;

    if (!i2c_device->HALOpen())
    {
        printf("Cannot open I2C peripheral\n");
        exit(-1);
    }
    else printf("I2C peripheral is opened\n");
    unsigned char data;
    if (!i2c_device->HALRead(0x38, 0xFF, 0, &data, ""))
    {
        printf("LCD is not found!\n");
        exit(-1);
    }
    else printf ("LCD is connected\n");
    lcd->LCDInit(i2c_device, 0x38, 20, 4);
    lcd->LCDBacklightOn();
    lcd->LCDCursorOn();

    ni::openni2_init();
    leftSign = imread("left.jpg",1);
    rightSign = imread("right.jpg",1);
}
double distance(Point p1, Point p2)
{
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}
void equalizeHistBGR(Mat &src, Mat &dst)
{
    if (src.channels() >= 3)
    {
        Mat ycrcb;

        cvtColor(src, ycrcb, CV_BGR2YUV);

        vector<Mat> channels;
        split(ycrcb, channels);

        equalizeHist(channels[0], channels[0]);

        Mat result;
        merge(channels, ycrcb);

        cvtColor(ycrcb, dst, CV_YUV2BGR);
    }
}
double similar(Mat &img1, Mat &img2)
{
    Mat hsv1, hsv2;
    resize(img1, hsv1, img1.size());
    equalizeHistBGR(hsv1, hsv1);
    cvtColor(hsv1, hsv1, COLOR_BGR2HSV);
    resize(img2, hsv2, hsv1.size());
    cvtColor(hsv2, hsv2, COLOR_BGR2HSV);
    int d = 0, s = 0;
    for (size_t i = 0; i < hsv1.cols; i++)
    {
        for (size_t j = 0; j < hsv1.rows; j++)
        {
            if (distance(Point(i, j), Point(hsv1.cols / 2, hsv1.rows / 2)) <= hsv1.rows / 2)
            {
                s++;
                if (hsv1.at<Vec3b>(j, i)[1] > 70 && hsv1.at<Vec3b>(j, i)[2] > 30)
                {
                    if (abs(hsv2.at<Vec3b>(j, i)[0] - hsv1.at<Vec3b>(j, i)[0]) < 15)
                    {
                        d++;
                    }
                }
                else
                {
                    if (abs(hsv2.at<Vec3b>(j, i)[2] - hsv1.at<Vec3b>(j, i)[2]) < 10)
                    {
                        d++;
                    }
                }
            }
        }
    }
    return (d * 1.0 / s);
}

void setControl(int speed,double angle)
{
    throttle_val=speed;
    theta=angle;
    api_set_FORWARD_control(pca9685,throttle_val);
    api_set_STEERING_control(pca9685,theta);
}
void putTextLCD(int x, int y,char *msg)
{
    lcd->LCDSetCursor(x,y);
    lcd->LCDPrintStr(msg);
}
void clearLCD()
{
    lcd->LCDClear();
}
bool nothingFrontSensor()
{
    unsigned int sensor_status = 0;
    for(int i=0; i<20; i++)
    {
        gpio->gpioGetValue(SENSOR, &sensor_status);
        if(sensor_status==1) return true;
    }
    return false;
}

bool isPressed(int PIN)
{
    unsigned int sensor_status = 0;
    for(int i=0; i<50; i++)
    {
        gpio->gpioGetValue(PIN, &sensor_status);
        if(sensor_status==0) return true;
    }
    return false;

}
int getLCDkey()
{
    if(isPressed(SW4_PIN)&&!sw4_stat)
    {
        sw4_stat=true;
    }
    if(!isPressed(SW4_PIN)&&sw4_stat)
    {
        sw4_stat=false;
        return 4;
    }
    if(isPressed(SW3_PIN)&&!sw3_stat)
    {
        sw3_stat=true;
    }
    if(!isPressed(SW3_PIN)&&sw3_stat)
    {
        sw3_stat=false;
        return 3;
    }
    if(isPressed(SW2_PIN)&&!sw2_stat)
    {
        sw2_stat=true;
    }
    if(!isPressed(SW2_PIN)&&sw2_stat)
    {
        sw2_stat=false;
        return 2;
    }
    if(isPressed(SW1_PIN)&&!sw1_stat)
    {
        sw1_stat=true;
    }
    if(!isPressed(SW1_PIN)&&sw1_stat)
    {
        sw1_stat=false;
        return 1;
    }
    return 0;
}


