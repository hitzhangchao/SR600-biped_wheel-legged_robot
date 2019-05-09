/***************************************************
 * SR600 motor control--for 4 motors
 * Functions:
 *      1. Launch 4 actuators;
 *      2. input target positoin;
 *      3. adjust by hand,we get balance motor target positon:(-12,12,-3.1,3.1),(-8 8 -0.35,0.35)
 * date:20190509
****************************************************/

#include <iostream>
#include "actuatorcontroller.h"
#include <thread>
#include <signal.h>
#include <atomic>
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
            if(!cin.fail())
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

void cmdOperation(double val_1,double val_2,double val_3,double val_4)
{
    //vector<uint8_t> idArray = controllerInst->getActuatorIdArray();
    ActuatorController *pController = ActuatorController::getInstance();
    vector<uint8_t> idArray = pController->getActuatorIdArray();
    pController->setPosition(idArray.at(0), val_1);
    pController->setPosition(idArray.at(1), val_2);
    pController->setPosition(idArray.at(2), val_3);
    pController->setPosition(idArray.at(3), val_4);
}

void processSignal(int sign)
{
    //cin.clear();
    ActuatorController::getInstance()->closeAllActuators();
    //delay some time to ensure close actuators successfully
    this_thread::sleep_for(std::chrono::milliseconds(200));
    bExit = true;
}

int main(int argc, char *argv[])
{
    ActuatorController::initController(Actuator::Via_Serialport);
    signal(SIGINT, processSignal);
    thread inputThread(keyboardInput);
    ActuatorController * pController = ActuatorController::getInstance();

    int nLaunchedActuatorCnt = 0;
    //connect controller operation signal
    //uint8_t nDevicedId is actuator ID,uint8_t operationType is operation type
    //operation type: Recgnize_Finished,Launch_Finished,Close_Finished,Save_Params_Finished
    int nOperationConnection = pController->m_sOperationFinished->s_Connect([&](uint8_t nDeviceId,uint8_t operationType){
        switch (operationType) {
        case Actuator::Recognize_Finished:
            if(pController->hasAvailableActuator())
            {
                vector<uint8_t> idArray = pController->getActuatorIdArray();
                cout << "Number of connected actuators:" << idArray.size() << endl;
                for (uint8_t id: idArray) {
                    if(pController->getActuatorAttribute(id,Actuator::ACTUATOR_SWITCH)==Actuator::ACTUATOR_SWITCH_OFF)
                    {
                        pController->launchActuator(id);
                    }
                }
            }
            break;
        case Actuator::Launch_Finished:
            //if all actuators havs launched, set profile position mode
            if(++nLaunchedActuatorCnt == pController->getActuatorIdArray().size())
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

    //connect controller error signal
    //uint8_t nDeviceId is actoator ID,uint16_t nErrorType is error type,string errorInfo is information of error
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

    //regognize the linking actuators automaticly
    pController->autoRecoginze();

    //controller process events loop--should not block the processEvent()
    while (!bExit)
    {
        ActuatorController::processEvents();
        if(inputFlag == true)
        {
            cmdOperation(value_1/1000.0,value_2/1000.0,value_3/1000.0,value_4/1000.0);
            inputFlag = false;
            value_1 = 0;
            value_2 = 0;
            value_3 = 0;
            value_4 = 0;
        }
    }

    //disconnect signal
    pController->m_sOperationFinished->s_Disconnect(nOperationConnection);
    pController->m_sError->s_Disconnect(nErrorConnection);


    inputThread.join();
    //getchar();
    return 0;
}
