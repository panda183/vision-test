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
#include "LaneDetector.h"
#include "SignDetector.h"

using namespace cv;
using namespace std;
using namespace EmbeddedFramework;

#define SW1_PIN 160
#define SW2_PIN 161
#define SW3_PIN 163
#define SW4_PIN 164
#define SENSOR 165
#define LED 166

int mode = 0;
GPIO *gpio;
PCA9685 *pca9685;
I2C *i2c_device;
LCDI2C *lcd;
Mat depthImg(240, 320, CV_16UC1),
    colorImg(240, 320, CV_8UC3);
bool sw1_stat, sw2_stat, sw3_stat, sw4_stat;
VideoWriter writer, writer2;
int videoidx = 0;
int angleArr[8];
float rootAngle;
void init();
void putTextLCD(int x, int y, char *msg);
void clearLCD();
bool nothingFrontSensor();
bool isPressed(int PIN);
int getLCDkey();
void Menu();
void RunCar();
void Destroy();
void setControl(int speed, double angle);
void Ready();

int main(int argc, char *argv[])
{
    init();
    Menu();
}

void Destroy()
{
    ni::openni2_destroy();
    setControl(0, 0);
    api_pwm_pca9685_release(pca9685);
    exit(1);
}
int nearAngle(float compass){
    compass+=180;
    int res=-1;
    for (int i = 0; i < 8; ++i)
    {
        if(i==0&&(compass>0&&compass<10)||(compass>350&&compass<360)){
            res=0;
            break;
        }else{
            if(abs(compass-i*45)<10){
                res=i;
                break;
            }
        }
    }
    return res;
}
void RunCar()
{

    utl::writer.open("birdViewLane " + to_string(videoidx) + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(640, 480), true);
    //writer2.open("./SignDetect "+to_string(videoidx)+".avi",CV_FOURCC('M','J','P','G'),25.0,Size(320,240),true);
    clearLCD();
    putTextLCD(0, 0, "Running");
    ld::hugLane = RIGHT;
    float P = 0, D = 0, angle = 0;
    int speed = 60;
    float curAngle;
    while (true)
    {
        curAngle=(getcompass+180)-rootAngle
        curAngle=curAngle<0?curAngle+360:curAngle;
        if(nearAngle(curAngle)!=-1){
            if(nearAngle!=last){
                angleArr[nearAngle]++;
                last=nearAngle;
                putTextLCD(0,2,to_string())
            }
        }
        int lcdKey = getLCDkey();
        if (lcdKey == 3 || !nothingFrontSensor())
        {
            Menu();
        }
        if (lcdKey == 1)
        {
            Destroy();
        } 
        auto cur_time = std::chrono::system_clock::now();
        ni::openni2_getmat(colorImg, depthImg);
        utl::splitGround(colorImg, depthImg);
        ld::findLane();
        sd::DetectSign(colorImg, depthImg);
        colorImg.copyTo(utl::videoFrame(cv::Rect(0,0,320, 240)));
        utl::writer.write(utl::videoFrame);
        if (sd::sign == -ld::hugLane)
        {
            ld::hugLane = -ld::hugLane;
        }
        float deltaTime = chrono::duration<double, milli>(std::chrono::system_clock::now() - cur_time).count();
        cout << deltaTime << endl;
        D = ((160.0 - ld::xCenterLane) - P) / deltaTime;
        P = (160.0 - ld::xCenterLane);
        angle = 4.5 * P + 10 * D;
        cout << angle << endl;
        setControl(speed, angle);
        if (speed < 90) speed++;
        if (sd::sign == STOP)
        {
            Menu();
        }
    }
    Menu();
}
void Ready()
{
    rootAngle=(getcompass+180);
    clearLCD();
    putTextLCD(0, 0, "Ready!!!");
    while (!nothingFrontSensor())
    {
        int key = getLCDkey();
        if (key == 3)
        {
            Menu();
        }
    }
    RunCar();
}

void Menu()
{
    videoidx++;
    if (utl::writer.isOpened())
    {
        utl::writer.release();
    }
    double angle = 0;
    setControl(0, 0);
    clearLCD();
    putTextLCD(0, 0, "Mode:Nghiem Tuc");
    //targetSpeed=55;
    while (true)
    {
        int key = getLCDkey();
        if (key == 4)
        {
            mode = (mode + 1) % 2;
            switch (mode)
            {
            case 0:
                putTextLCD(5, 0, "Nghiem Tuc");
                break;
            case 1:
                putTextLCD(5, 0, "Thi cu =))");
                break;
            default:
                break;
            }
        }
        if (key == 3)
        {
            Ready();
            break;
        }
        if (key == 1)
            Destroy();
    }
}
void init()
{
    //=========== Init  =======================================================
    ////////  Init PCA9685 driver   ///////////////////////////////////////////
    pca9685 = new PCA9685();
    api_pwm_pca9685_init(pca9685);
    if (pca9685->error < 0)
        return;
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
    else
        printf("I2C peripheral is opened\n");
    unsigned char data;
    if (!i2c_device->HALRead(0x38, 0xFF, 0, &data, ""))
    {
        printf("LCD is not found!\n");
        exit(-1);
    }
    else
        printf("LCD is connected\n");
    lcd->LCDInit(i2c_device, 0x38, 20, 4);
    lcd->LCDBacklightOn();
    lcd->LCDCursorOn();

    ni::openni2_init();
    utl::readGroundPlane();
    utl::getTransformMatrix();
    sd::init();
}

void setControl(int speed, double angle)
{
    api_set_FORWARD_control(pca9685, speed);
    api_set_STEERING_control(pca9685, angle);
}

void putTextLCD(int x, int y, char *msg)
{
    lcd->LCDSetCursor(x, y);
    lcd->LCDPrintStr(msg);
}

void clearLCD()
{
    lcd->LCDClear();
}

bool nothingFrontSensor()
{
    unsigned int sensor_status = 0;
    for (int i = 0; i < 20; i++)
    {
        gpio->gpioGetValue(SENSOR, &sensor_status);
        if (sensor_status == 1)
            return true;
    }
    return false;
}

bool isPressed(int PIN)
{
    unsigned int sensor_status = 0;
    for (int i = 0; i < 20; i++)
    {
        gpio->gpioGetValue(PIN, &sensor_status);
        if (sensor_status == 0)
            return true;
    }
    return false;
}

int getLCDkey()
{
    if (isPressed(SW4_PIN) && !sw4_stat)
    {
        sw4_stat = true;
    }
    if (!isPressed(SW4_PIN) && sw4_stat)
    {
        sw4_stat = false;
        return 4;
    }
    if (isPressed(SW3_PIN) && !sw3_stat)
    {
        sw3_stat = true;
    }
    if (!isPressed(SW3_PIN) && sw3_stat)
    {
        sw3_stat = false;
        return 3;
    }
    if (isPressed(SW2_PIN) && !sw2_stat)
    {
        sw2_stat = true;
    }
    if (!isPressed(SW2_PIN) && sw2_stat)
    {
        sw2_stat = false;
        return 2;
    }
    if (isPressed(SW1_PIN) && !sw1_stat)
    {
        sw1_stat = true;
    }
    if (!isPressed(SW1_PIN) && sw1_stat)
    {
        sw1_stat = false;
        return 1;
    }
    return 0;
}
