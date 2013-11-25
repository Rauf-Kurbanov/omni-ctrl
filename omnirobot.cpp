#include <QDebug>

#include "omnirobot.h"

#include <cmath>
#include <linux/input.h>

const float gyroConvConst = 838.736454707;
#define rt_SATURATE(sig,ll,ul)         (((sig) >= (ul)) ? (ul) : (((sig) <= (ll)) ? (ll) : (sig)) )
const float c1 = sqrt(3) * 0.5;
const float c2 = 0.5;

OmniRobot::OmniRobot(QThread *guiThread):
    brick(*guiThread),
    gyrolast(0),
    alpha(0.0),
    Mt(3,3),
    Rot(2,2),
    cmd(2),
    pwm(3),
    mov(3)
{
    qDebug() << "INIT_MODE";

    init();
    connect(brick.gamepad(), SIGNAL(trikGamepad_button(int,int)), this, SLOT(gamepadButton(int, int)));
    connect(brick.gamepad(), SIGNAL(trikGamepad_pad(int,int,int)), this, SLOT(gamepadPad(int,int,int)));
    connect(brick.gamepad(), SIGNAL(trikGamepad_padUp(int)), this, SLOT(gamepadPadUp(int)));

    connect(brick.keys(), SIGNAL(buttonPressed(int,int)), this, SLOT(getButton(int,int)));
}

void OmniRobot::init()
{
    period = 200;
    xw = 75.0; //mm
    yw = 75.0; //mm
    Dw = 50.0; //mm

    vector<float> u1 (2); u1(0) =   0; u1(1) =   1;
    vector<float> u2 (2); u2(0) =  c1; u2(1) = -c2;
    vector<float> u3 (2); u3(0) = -c1; u3(1) = -c2;

    vector<float> n1 (2); n1(0) =  -1; n1(1) =   0;
    vector<float> n2 (2); n2(0) =  c2; n2(1) =  c1;
    vector<float> n3 (2); n3(0) =  c1; n3(1) = -c2;

    vector<float> b1 (2); b1(0) =     0; b1(1) =     yw;
    vector<float> b2 (2); b2(0) = c1*xw; b2(1) = -c2*yw;
    vector<float> b3 (2); b3(0) =-c1*xw; b3(1) = -c2*yw;

    Mt(0,0) = n1(0); Mt(0,1) = n1(1); Mt(0,2) = b1(0)*u1(0) + b1(1)*u1(1);
    Mt(1,0) = n2(0); Mt(1,1) = n2(1); Mt(1,2) = b2(0)*u2(0) + b2(1)*u2(1);
    Mt(2,0) = n3(0); Mt(2,1) = n3(1); Mt(2,2) = b3(0)*u3(0) + b3(1)*u3(1);

    Mt = -1 * Mt;

    cmd(0) = 0.0; cmd(1) = 0.0;
    pwm(0) = 0.0; pwm(1) = 0.0; pwm(2) = 0.0;
    mov(0) = 0.0; mov(1) = 0.0; mov(2) = 0.0;

    omniState = INIT_MODE;
    movementMode = ROTATE_MODE;
    power = 20;
    pplus = 1;

    gyrolast = brick.gyroscope()->read()[2];
}

void OmniRobot::gamepadPad(int pad, int vx, int vy)
{
    if (pad != 1) return;
    cmd(0) = vx;
    cmd(1) = vy;
}

void OmniRobot::gamepadPadUp(int pad)
{
    if (pad != 1) return;
    cmd(0) = 0.0;
    cmd(1) = 0.0;
    brick.stop();
}

void OmniRobot::gamepadButton(int button, int pressed)
{
    if (pressed == 0) return;

    switch (omniState)
    {
    case INIT_MODE:
        switch (button)
        {
        case 1: movementMode = ROTATE_MAX_MODE; break;
        case 2: movementMode = ROTATE_POINT_MODE; break;
        case 3: movementMode = ROTATE_MODE; break;
        case 5: movementMode = ANDROID_MODE; period = 20; break;
        }
        omniState = CONTROL_MODE;
        startControl();
        break;
    case CONTROL_MODE:
        qDebug() << "INIT_MODE";
        init();
        omniState = INIT_MODE;
        brick.stop();
        timer.stop();
        disconnect(&timer, SIGNAL(timeout()), this, SLOT(omniControl()));
        break;
    default:
        brick.stop();
        timer.stop();
        disconnect(&timer, SIGNAL(timeout()), this, SLOT(omniControl()));
    }

}

void OmniRobot::getButton(int code, int value)
{
    if (value != 1)
        return;

    qDebug() << "INIT_MODE";
    omniState = INIT_MODE;
    init();
    brick.stop();
    timer.stop();
    disconnect(&timer, SIGNAL(timeout()), this, SLOT(omniControl()));
}

void OmniRobot::startControl()
{
    brick.stop();
    timer.stop();
    qDebug() << "CONTROL_MODE";
    connect(&timer, SIGNAL(timeout()), this, SLOT(omniControl()));
    timer.start(period);
}

void OmniRobot::omniControl()
{
    switch (movementMode)
    {
    case ROTATE_POINT_MODE:
        rotatepoint();
        break;
    case ROTATE_MODE:
        rotate();
        break;
    case ROTATE_MAX_MODE:
        rotatemax();
        break;
    case ANDROID_MODE:
        androidmode();
        break;
    default:
        brick.stop();
        break;
    }
}

void OmniRobot::androidmode()
{
    qDebug("cmd: %f, %f", cmd(0), cmd(1));

    QVector<int> temp = brick.gyroscope()->read();
    alpha += (temp[0] + gyrolast) * 0.5 * 0.001 * period;

    float calpha = 0;// Cos(alpha);
    float salpha = 0;//Sin(alpha);
    Rot(0,0) = calpha; Rot(0,1) = -salpha;
    Rot(1,0) = salpha; Rot(1,1) = calpha;
    vector<float> t = prod(Rot, cmd);

    mov(0) = t(0); mov(1) = t(0); mov(2) = 0.0;

    pwm = prod(Mt, mov);
    pwm = pwm * 2.0 / Dw ;

    qDebug("pwm: %f, %f, %f", pwm(0), pwm(1), pwm(2));

    brickPower();
}

void OmniRobot::brickPower()
{
    int m1 = rt_SATURATE((int)pwm(0), -100, 100);
    int m2 = rt_SATURATE((int)pwm(1), -100, 100);
    int m4 = rt_SATURATE((int)pwm(2), -100, 100);
    brick.powerMotor("1")->setPower(m1);
    brick.powerMotor("2")->setPower(m2);
    brick.powerMotor("4")->setPower(m4);
}

void OmniRobot::rotate()
{
    if (power == 20)
        pplus = 2;
    if (power == 100)
        pplus = -2;
    power = power + pplus;

    brick.powerMotor("1")->setPower(power);
    brick.powerMotor("2")->setPower(power);
    brick.powerMotor("3")->setPower(power);
    brick.powerMotor("4")->setPower(power);
}

void OmniRobot::rotatepoint()
{
    brick.powerMotor("2")->setPower(100);
    brick.powerMotor("3")->setPower(100);
    brick.powerMotor("4")->setPower(100);
}

void OmniRobot::rotatemax()
{
    brick.powerMotor("1")->setPower(100);
    brick.powerMotor("2")->setPower(100);
    brick.powerMotor("3")->setPower(100);
    brick.powerMotor("4")->setPower(100);
}
