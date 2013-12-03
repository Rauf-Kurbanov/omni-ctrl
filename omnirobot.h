#pragma once

#include <QObject>
#include <QTimer>
#include <QMatrix3x3>
#include <QVector3D>
#include <trikControl/brick.h>

using namespace trikControl;

class OmniRobot : public QObject
{
    Q_OBJECT
public:
    explicit OmniRobot(QThread *guiThread);

protected:
    void init();

    void brickPower();

    void startControl();
    void androidmode();
    
signals:
    
public slots:

private slots:
    void getButton(int code, int value);
    void gamepadButton(int button, int pressed);
    void omniControl();
    void gamepadPad(int pad, int vx, int vy);
    void gamepadPadUp(int pad);

private:
    int period;
    enum { INIT_MODE,
           CONTROL_MODE
    } omniState;

    enum { ROTATE_MODE,
           ROTATE_MAX_MODE,
           ROTATE_POINT_MODE,
           ANDROID_MODE
    } movementMode;

    Brick   brick;
    QTimer  timer;

    qreal Dw;
    qreal xw;
    qreal yw;

    int gyrolast;
    int gyroerror;
    qreal alpha;

    QMatrix3x3 Mt;
    QVector3D cmd;
    QVector3D pwm;
    //matrix<float> Rot;
    //vector<float> cmd;
    //vector<float> pwm;
    //vector<float> mov;
};
