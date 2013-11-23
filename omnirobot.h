#pragma once

#include <QObject>
#include <QTimer>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <trikControl/brick.h>

const float xw = 0.165;
const float yw = 0.132;

using namespace trikControl;
using namespace boost::numeric::ublas;

class OmniRobot : public QObject
{
    Q_OBJECT
public:
    explicit OmniRobot(QThread *guiThread);

protected:
    void brickPower();

    void startControl();
    void androidmode();
    void rotatepoint();
    void rotatemax();
    void rotate();

    void init();
    
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
    int power;
    int pplus;

    int Dw;
    int xw;
    int yw;

    int gyrolast;
    float alpha;

    matrix<float> Mt;
    matrix<float> Rot;
    vector<float> cmd;
    vector<float> pwm;
    vector<float> mov;
};
