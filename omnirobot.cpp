#include <QDebug>

#include "omnirobot.h"
#include <QVector2D>
#include <cmath>

const qreal gyroConvConst = 838.736454707;
#define rt_SATURATE(sig,ll,ul)         (((sig) >= (ul)) ? (ul) : (((sig) <= (ll)) ? (ll) : (sig)) )
const qreal c1 = sqrt(3) * 0.5;
const qreal c2 = 0.5;

OmniRobot::OmniRobot(QThread *guiThread):
    brick(*guiThread),
    gyrolast(0),
    alpha(0.0),
    Mt(),
    cmd()
{
    qDebug() << "OMNI_STARTS";

    init();
    connect(brick.gamepad(), SIGNAL(trikGamepad_button(int,int)), this, SLOT(gamepadButton(int, int)));
    connect(brick.gamepad(), SIGNAL(trikGamepad_pad(int,int,int)), this, SLOT(gamepadPad(int,int,int)));
    connect(brick.gamepad(), SIGNAL(trikGamepad_padUp(int)), this, SLOT(gamepadPadUp(int)));

    connect(brick.keys(), SIGNAL(buttonPressed(int,int)), this, SLOT(getButton(int,int)));
}

void OmniRobot::init()
{
    period = 1000;
    xw = 110.0; //mm
    yw = 110.0; //mm
    Dw = 0.02; // 1/50mm

    QVector2D u1(0.0, 1.0);
    QVector2D u2(c1, -c2);
    QVector2D u3(-1.0, 0.0);

    QVector2D n1(-1.0, 0.0);
    QVector2D n2(c2, c1);
    QVector2D n3(c1, -c2);

    QVector2D b1(0.0, yw);
    QVector2D b2(c1*xw, -c2*yw);
    QVector2D b3(-c1*xw, -c2*yw);

    Mt(0,0) = n1.x(); Mt(0,1) = n1.y(); Mt(0,2) = b1.x()*u1.x() + b1.y()*u1.y();
    Mt(1,0) = n2.x(); Mt(1,1) = n2.y(); Mt(1,2) = b2.x()*u2.x() + b2.y()*u3.y();
    Mt(2,0) = n3.x(); Mt(2,1) = n3.y(); Mt(2,2) = b3.x()*u3.x() + b3.y()*u3.y();

    Mt *= -1.0;

    pwm.setX(0.0); pwm.setY(0.0); pwm.setZ(0.0);
    cmd.setX(0.0); cmd.setY(0.0); cmd.setZ(0.0);

    omniState = INIT_MODE;
    movementMode = ROTATE_MODE;
    gyroerror = brick.gyroscope()->read()[2];
}

void OmniRobot::gamepadPad(int pad, int vx, int vy)
{
    if (pad != 1) return;
    cmd.setX((qreal)vx);
    cmd.setY((qreal)vy);
}

void OmniRobot::gamepadPadUp(int pad)
{
    if (pad != 1) return;
    cmd.setX(0.0);
    cmd.setY(0.0);
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
        break;
    case ROTATE_MODE:
        break;
    case ROTATE_MAX_MODE:
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
    pwm.setX(Mt(0,0)*cmd.x() + Mt(0,1)*cmd.y() + Mt(0,2)*cmd.z());
    pwm.setY(Mt(1,0)*cmd.x() + Mt(1,1)*cmd.y() + Mt(1,2)*cmd.z());
    pwm.setZ(Mt(2,0)*cmd.x() + Mt(2,1)*cmd.y() + Mt(2,2)*cmd.z());

    pwm *= 2.0 * 20.0 * Dw ;

    //qDebug("pwm: %f, %f, %f", pwm.x(), pwm.y(), pwm.z());

    brickPower();

    int gyronew = brick.gyroscope()->read()[2];
    alpha += (gyronew - gyrolast) * 0.001 * period;
    gyrolast = gyronew;

    qDebug("aplha: %f", alpha);
    /*
    float calpha = 0;//Cos(alpha);
    float salpha = 0;//Sin(alpha);
    Rot(0,0) = calpha; Rot(0,1) = -salpha;
    Rot(1,0) = salpha; Rot(1,1) = calpha;
    vector<float> t = prod(Rot, cmd);
    */
    /*
    mov(0) = cmd(0); mov(1) = cmd(1); mov(2) = 0.0;

    pwm = prod(Mt, mov);
    pwm = pwm * 2.0 / Dw ;

    qDebug("pwm: %f, %f, %f", pwm(0), pwm(1), pwm(2));

    brickPower();
    */
}

void OmniRobot::brickPower()
{

    int m1 = rt_SATURATE((int)pwm.x(), -100, 100);
    int m2 = rt_SATURATE((int)pwm.y(), -100, 100);
    int m4 = rt_SATURATE((int)pwm.z(), -100, 100);
    brick.powerMotor("1")->setPower(m1);
    brick.powerMotor("2")->setPower(m2);
    brick.powerMotor("4")->setPower(m4);

}
