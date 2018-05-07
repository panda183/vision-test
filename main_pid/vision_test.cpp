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
Mat depthImg(480, 640, CV_16UC1),
    colorImg(480, 640, CV_8UC3);
bool sw1_stat, sw2_stat, sw3_stat, sw4_stat;
VideoWriter writer, writer2;
int videoidx = 0;

void init();
void putTextLCD(int x, int y, char *msg);
void clearLCD();
bool nothingFrontSensor();
bool isPressed(int PIN);
int getLCDkey();
void Menu();
bool RunCar();
void Destroy();
void setControl(int speed, double angle);
void Ready();

int main(int argc, char *argv[])
{
    init();
    Menu();
    Destroy();
}

void Destroy()
{
    ni::openni2_destroy();
}

bool RunCar()
{
    //writer.open("./birdViewLane "+to_string(videoidx)+".avi",CV_FOURCC('M','J','P','G'),25.0,Size(320,240),true);
    //writer2.open("./SignDetect "+to_string(videoidx)+".avi",CV_FOURCC('M','J','P','G'),25.0,Size(320,240),true);
    clearLCD();
    putTextLCD(0, 0, "Running");

    while (true)
    {
        int key = getLCDkey();
        if (key == 3 || !nothingFrontSensor())
        {
            Menu();
        }
        auto cur_time = std::chrono::system_clock::now();
        ni::openni2_getmat(colorImg, depthImg);
        imshow("color",colorImg);
        // setControl(speed, angle);
        cout<< chrono::duration<double, milli> (std::chrono::system_clock::now()-cur_time).count()<<endl;
        if (waitKey(1) == 27) break;
    }
    return true;
}
void Ready()
{
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
    if (RunCar()) Menu();
}

void Menu()
{
    //videoidx++;
    if (writer.isOpened())
    {
        writer.release();
    }
    if (writer2.isOpened())
    {
        writer2.release();
    }
    double angle = 0;
    setControl(0, 0);
    clearLCD();
    putTextLCD(0, 0, "Mode:Nghiem Tuc");
    //targetSpeed=55;
    do
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
        }

    } while (1);
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
    for (int i = 0; i < 20; i++ )
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
    for (int i = 0; i <  20; i++)
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
