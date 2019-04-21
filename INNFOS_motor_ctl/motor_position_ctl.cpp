/*
*profile position模式下，终端里输入对应电机的position value,就可以实现四个电机的不同位置控制
*/
#include <iostream>
#include "actuatorcontroller.h"
#include <thread>
#include <signal.h>
#include <atomic>    //所谓的原子操作，在多个线程访问同一个全局资源的时候，确保所有其他的线程都不在同一时间内访问相同的资源。
#include <string>
using namespace std;

atomic_bool bExit{false};
atomic_bool inputFlag{false};
atomic_int value_1{0};
atomic_int value_2{0};
atomic_int value_3{0};
atomic_int value_4{0};

void keyboardInput()
{
    double a;
    double b;
    double c;
    double d;
    while (!bExit)
    {
        if(true)
        {
            cin >> a;
            cin >> b;
            cin >> c;
            cin >> d;
            if(!cin.fail())    //fail()是流对象的成员函数，若刚进行的操作失败，则返回真，否则返回假
            {
                value_1 = a*1000;
                value_2 = b*1000;
                value_3 = c*1000;
                value_4 = d*1000;

                inputFlag = true;
            }
        }
    }
}

void cmdOperation(double value_1,double value_2,double value_3,double value_4)
{
    //vector<uint8_t> idArray = controllerInst->getActuatorIdArray();
    ActuatorController *pController = ActuatorController::getInstance();
    vector<uint8_t> idArray = pController->getActuatorIdArray();
        pController->setPosition(idArray.at(0), value_1);
        pController->setPosition(idArray.at(1), value_2);
        pController->setPosition(idArray.at(2), value_3);
        pController->setPosition(idArray.at(3), value_4);
}

void processSignal(int sign)
{
    //cin.clear();
    ActuatorController::getInstance()->closeAllActuators();
    //延时保证关机成功
    this_thread::sleep_for(std::chrono::milliseconds(200));
    bExit = true;
}

int main(int argc, char *argv[])
{
    //初始化控制器
    ActuatorController::initController(Actuator::Via_Serialport);
    signal(SIGINT, processSignal);
    thread inputThread(keyboardInput);
    ActuatorController * pController = ActuatorController::getInstance();

    int nLaunchedActuatorCnt = 0;
    //关联控制器的操作信号
    int nOperationConnection = pController->m_sOperationFinished->s_Connect([&](uint8_t nDeviceId,uint8_t operationType){
        switch (operationType) {
        case Actuator::Recognize_Finished://自动识别完成
            if(pController->hasAvailableActuator())
            {
                vector<uint8_t> idArray = pController->getActuatorIdArray();
                cout << "Number of connected actuators:" << idArray.size() << endl;
                for (uint8_t id: idArray) {
                    if(pController->getActuatorAttribute(id,Actuator::ACTUATOR_SWITCH)==Actuator::ACTUATOR_SWITCH_OFF)
                    {//如果执行器处于关机状态，启动执行器
                        pController->launchActuator(id);
                    }
                }
            }
            break;
        case Actuator::Launch_Finished:
            if(++nLaunchedActuatorCnt == pController->getActuatorIdArray().size())//所有执行器都已启动完成,同时将电机上电（profile position mode）
            {
                vector<uint8_t> idArray = pController->getActuatorIdArray();
                pController->activateActuatorMode(idArray,Actuator::Mode_Profile_Pos);
                cout << "All actuators have launched!"  << endl;
            }
            break;
        default:
            break;
        }
    });

    //关联错误信号
    int nErrorConnection = pController->m_sError->s_Connect([=](uint8_t nDeviceId,uint16_t nErrorType,string errorInfo){
        if(nDeviceId==0)
        {
            cout << "Error: " << (int)nErrorType << " " << errorInfo << endl;
        }
        else
        {
            cout << "Actuator " << (int)nDeviceId << " " <<"error " << (int)nErrorType << " " << errorInfo << endl;
        }
    });

    //自动识别已连接执行器
    pController->autoRecoginze();

    //执行控制器事件循环
    while (!bExit)
    {
        ActuatorController::processEvents();
        if(inputFlag==true)
        {
            cmdOperation(value_1/1000.0,value_2/1000.0,value_3/1000.0,value_4/1000.0);
            inputFlag = false;
            value_1 = 0;
            value_2 = 0;
            value_3 = 0;
            value_4 = 0;
        }
    }

    //断开信号连接
    pController->m_sOperationFinished->s_Disconnect(nOperationConnection);
    pController->m_sError->s_Disconnect(nErrorConnection);


    inputThread.join();
    //getchar();
    return 0;
}
