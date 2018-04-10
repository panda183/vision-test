#include <iostream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <ctime>

using namespace std;
using namespace cv;
//Client side
//create a message buffer
char msg[100000];
int clientSd;
Mat leftSign = imread("left.jpg");
Mat rightSign = imread("right.jpg");
bool bamlephai = true;
int xLeftLane=100,xRightLane=210;
clock_t lastTimeSignDetected;

bool ConnectToServer(char *serverIp, int port)
{
	//setup a socket and connection tools
	struct hostent *host = gethostbyname(serverIp);
	sockaddr_in sendSockAddr;
	bzero((char *)&sendSockAddr, sizeof(sendSockAddr));
	sendSockAddr.sin_family = AF_INET;
	sendSockAddr.sin_addr.s_addr = inet_addr(inet_ntoa(*(struct in_addr *)*host->h_addr_list));
	sendSockAddr.sin_port = htons(port);
	clientSd = socket(AF_INET, SOCK_STREAM, 0);
	//try to connect...
	int status = connect(clientSd, (sockaddr *)&sendSockAddr, sizeof(sendSockAddr));
	if (status < 0)
	{
		cout << "Error connecting to socket!" << endl;
		return false;
	}
	cout << "Connected to the server!" << endl;
	return true;
}
Mat GetImageFromServer()
{
	memset(&msg, 0, sizeof(msg)); //clear the buffer
	recv(clientSd, (char *)&msg, sizeof(msg), 0);

	vector<char> arr(msg, msg + sizeof(msg) - 1);
	Mat decodedImage = imdecode(arr, CV_LOAD_IMAGE_COLOR);
	return decodedImage;
}

void SendDataToServer(float torque, float angle)
{
	memset(&msg, 0, sizeof(msg)); //clear the buffer
	string data = to_string(torque) + "|" + to_string(angle);
	//cout << data << endl;
	send(clientSd, data.c_str(), data.length(), 0);
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
	warpPerspective(input, output, lambda, output.size());
	cvtColor(output, output, COLOR_BGR2GRAY);
	threshold(output, output, 100, 255, CV_THRESH_BINARY);
	//imshow("Output", output);
	return output;
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
int recognizeSign(Mat &sign)
{
	double p1 = similar(sign, leftSign);
	double p2 = similar(sign, rightSign);
	if (p1 > p2)
		return 0;
	return 1;
}
double angleCal(Point p){
		float angle;
		if (p.x != 160)
		{
			double k = (p.y * 1.0f - 240) / (p.x * 1.0f - 160);
			angle = -atan(k) * 180 / 3.14159265;
		}
		else
		{
			angle = 90;
		}
		int dau = angle / abs(angle);
		angle = (90 - abs(angle)) * dau;
	return angle;
}
Point getTargetPoint(Mat &birdViewImg, bool bamlephai)
{
	int yCenter = 200;
	int offSet=50;
	Point target = Point(birdViewImg.cols / 2, birdViewImg.rows - 1);
	if (bamlephai)
	{
		for (size_t j = birdViewImg.cols - 1; j > 2; j--)
		{
			if ((int)birdViewImg.at<uchar>(yCenter, j) == 0 && (int)birdViewImg.at<uchar>(yCenter, j - 1) == 255)
			{
				target = Point(j - offSet, yCenter);
					break;
				
				
			}
		}
	}
	else
	{
		for (size_t j = 0; j < birdViewImg.cols - 2; j++)
		{
			if ((int)birdViewImg.at<uchar>(yCenter, j) == 0 && (int)birdViewImg.at<uchar>(yCenter, j + 1) == 255)
			{
				target = Point(j + offSet, yCenter);
				break;
				
			}
		}
	}
	line(birdViewImg,Point(160,239),target,Scalar(255));
	return target;
}
void DetectSign(Mat &src)
{

	Mat hsv, gray;
	Rect roiDetect = Rect(100, 25, 150, 40);
	cvtColor(src(roiDetect), hsv, COLOR_BGR2HSV);
	Scalar min = Scalar(90, 150, 85);   //HSV VALUE
	Scalar max = Scalar(130, 255, 255); //HSV VALUE
	inRange(hsv, min, max, gray);
	dilate(gray, gray, Mat(), Point(-1, -1), 6, 1, 1);
	//erode(gray, gray, Mat(), Point(-1, -1), 5, 1, 1);
	vector<vector<Point>> contours;
	findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	for (int i = 0; i < contours.size(); i++)
	{
			Rect r = boundingRect(contours[i]);
			if (abs((r.width * 1.0 / r.height) - 1) < 0.1 && r.width>20 && r.width<30)
			{
				r.x += roiDetect.x + 6;
				r.y += roiDetect.y + 6;
				r.width -= 6 * 2;
				r.height -= 6 * 2;
				rectangle(src, r, Scalar(0, 0, 255));
				Mat matsign = src(r);
				bamlephai = recognizeSign(matsign);
				if(bamlephai){
					cout<<"phai"<<endl;

				}else{
					cout<<"trai"<<endl;
				}
				//lastTimeSignDetected = clock();
				imshow("range", matsign);
			}
	}
	imshow("src", src);
}
int main(int argc, char *argv[])
{
	float _p=0;
	if (argc != 3)
	{
		cerr << "Usage: ip_address port" << endl;
		exit(0);
	}
	char *serverIp = argv[1];
	int port = atoi(argv[2]);

	if (!ConnectToServer(serverIp, port))
	{
		return 0;
	}

	Mat img = GetImageFromServer();
	DetectSign(img);
	SendDataToServer(0, 0);
	do
	{
		clock_t start = clock();
		img = GetImageFromServer();
		if (!img.data)
		{
			cout << "loi nhan anh";
			if (msg[0] == 'c' && msg[1] == 'l')
			{
				close(clientSd);
				return 0;
			}
			continue;
		}
		 if ((clock() - lastTimeSignDetected) / 1000 > 500){
			DetectSign(img);
		 }
		
		int checkRange=70;
		Mat laneImg = birdView(img);

		//float angle=angleCal(getTargetPoint(laneImg,bamlephai));
		//cout<<angle<<endl;
		int xLaneL=-1,xLaneR=-1;
		for(int y=laneImg.rows-1;y>=100;y-=5){
			if(xLaneL==-1&&!bamlephai){
				for(int i=xLeftLane-checkRange/2;i<xLeftLane+checkRange/2;i++){
					if(i>=0&&i<=laneImg.cols){
						if(laneImg.at<uchar>(y,i)==255){
							xLaneL=i;
							break;
						}
					}
				}
				xLaneR=xLaneL+90;
			}
			if(xLaneR==-1&&bamlephai){
				
				for(int i=xRightLane-checkRange/2;i<xRightLane+checkRange/2;i++){
					if(i>=0&&i<=laneImg.cols){
						if(laneImg.at<uchar>(y,i)==255){
							xLaneR=i;
							
							break;
						}
					}
				}
				
				xLaneL=xLaneR-90;
			}
			line(laneImg,Point(xLeftLane-checkRange/2,y),Point(xLeftLane+checkRange/2,y),Scalar(255));
			line(laneImg,Point(xRightLane-checkRange/2,y),Point(xRightLane+checkRange/2,y),Scalar(255));
			if(xLaneR!=-1&&xLaneL!=-1){
				xLeftLane=xLaneL;
				xRightLane=xLaneR;
				break;
			}
		}
		int dau=(bamlephai-0.5)*2;
		int tmp=(xLeftLane+xRightLane)/2+dau;
		float dP=(tmp-160)-_p;
		float _d=dP*0;
		_p=(160-tmp)*0.3;
		cout<<_p<<":"<<_d<<endl;
		float angle=-(_p+_d);
		circle(laneImg,Point(tmp,239),3,Scalar(255),2);
		imshow("birdView", laneImg);		
		int keyCode = waitKey(1);
		usleep(2000);
		if (keyCode == 27)
			break;
		SendDataToServer(7, angle);
		int FPS = 1000000 / (clock() - start);
		//cout << "FPS:" << FPS << endl;
	} while (1);
	close(clientSd);
	return 0;
}
