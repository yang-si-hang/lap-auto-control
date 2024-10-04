#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
//***********************
#include <cstdio>
//***********************
//#include <Windows.h>
#include <unistd.h>
#define Sleep(x) (usleep (x*1000))
#include "../include/hps_force_sensor/hps_ft_lib.h"

void mFTCallback(const int handle,const double ft_data[6],const hps_ft_info ft_info);
void test_handle();
void test_rs485();
void test_ethernet_udp();
void handle_test();
//-----------------------------------------------------
void mFTCallback(const int handle,const double ft_data[6],const hps_ft_info ft_info){
    std::cout << ft_info.IPOC << std::endl;
    std::cout << "\t" << ft_info.IPOC << "\t" << int(ft_info.code) << "\t" << ft_info.code_info;
    std::cout << "[\t";
    for (int j = 0; j < 6; j++)
    {
        std::cout << ft_data[j];
        std::cout << "\t";
    }
    std::cout << "]" << std::endl;
}

void test_handle()
{

//    double ft_data[6];
//    hps_ft_info ft_info;
    HPS_FT_HANDLE ft_handle[10];

    for (size_t i = 0; i < 10; i++)
    {
        ft_handle[i]=hps_ft_createHandle(HpsSensorCommEnum::RS485);
        std::cout << ft_handle[i] << std::endl;
    }


    size_t num_ft = 0;
    for (size_t i = 0; i < 10; i++)
    {
        if(hps_ft_initial(ft_handle[i],1500000,"")!=HPS_FT_SUCCESS){
            num_ft = i;
            std::cout <<"Automatic connection "<< num_ft <<" desk equipment!"<< std::endl;
            break;
        }
    }

    for (size_t i = num_ft; i < 10; i++)
    {
        hps_ft_deleteHandle(&ft_handle[i]);
        std::cout << num_ft <<ft_handle[i] << "hps_ft_deleteHandle" << std::endl;
    }

    for (size_t j = 0; j < num_ft; j++)
    {
        if(hps_ft_setLowPassFilter(ft_handle[j],0)!=HPS_FT_SUCCESS){
            std::cout << ft_handle[j] <<" hps_ft_setLowPassFilter ERROR!"<< std::endl;
        }

        if(hps_ft_zero(ft_handle[j])!=HPS_FT_SUCCESS){
            std::cout << ft_handle[j] <<" hps_ft_zero ERROR!"<< std::endl;
        }

        Sleep(100);
        if(hps_ft_registerCallback(ft_handle[j],mFTCallback)!=HPS_FT_SUCCESS){
            std::cout << ft_handle[j] <<" hps_ft_register_callback ERROR!"<< std::endl;
        }
    }
    std::cout << " callback run!"<< std::endl;
    Sleep(1000);
    std::cout << " callback end!"<< std::endl;

    for (size_t i = 0; i < num_ft; i++)
    {
        if(hps_ft_uninitial(ft_handle[i])!=HPS_FT_SUCCESS){
            std::cout << ft_handle[i] <<" hps_ft_uninitial ERROR!"<< std::endl;
        }

        hps_ft_deleteHandle(&ft_handle[i]);
        std::cout << num_ft <<ft_handle[i] << "hps_ft_deleteHandle" << std::endl;
    }
}

void test_rs485()
{
    double ft_data[6];
    hps_ft_info ft_info;
    HPS_FT_HANDLE ft_rs485_handle;
    ft_rs485_handle=hps_ft_createHandle(HpsSensorCommEnum::RS485);

    if(hps_ft_initial(ft_rs485_handle,115200,"")!=HPS_FT_SUCCESS){
        std::cout << ft_rs485_handle <<" hps_ft_initial ERROR!"<< std::endl;
    }

    if(hps_ft_setLowPassFilter(ft_rs485_handle,0)!=HPS_FT_SUCCESS){
        std::cout << ft_rs485_handle <<" hps_ft_setLowPassFilter ERROR!"<< std::endl;
    }

    if(hps_ft_zero(ft_rs485_handle)!=HPS_FT_SUCCESS){
        std::cout << ft_rs485_handle <<" hps_ft_zero ERROR!"<< std::endl;
    }

    int deviceModeInfo;
    if(hps_ft_getDeviceModeInfo2(ft_rs485_handle,&deviceModeInfo)==HPS_FT_SUCCESS){
        std::cout << ft_rs485_handle <<"hps_ft_getDeviceModeInfo2 "<< deviceModeInfo << std::endl;
    }

//    if(hps_ft_setStopSample(ft_rs485_handle)!=HPS_FT_SUCCESS){
//        std::cout << ft_rs485_handle <<" hps_ft_zero ERROR!"<< std::endl;
//    }

    Sleep(100);

    for (size_t i = 0; i < 20; i++)
    {
        if(hps_ft_getData2(ft_rs485_handle,ft_data)==HPS_FT_SUCCESS){
            std::cout << "[\t";
            for (int j = 0; j < 6; j++)
            {
                std::cout << ft_data[j];
                std::cout << "\t";
            }
            std::cout << "]" << std::endl;
            Sleep(100);
        }
        else{
            std::cout << ft_rs485_handle <<" hps_ft_getData ERROR!"<< std::endl;
            break;
        }
    }

    for (size_t i = 0; i < 20; i++)
    {
        if(hps_ft_getData(ft_rs485_handle,ft_data, ft_info)==HPS_FT_SUCCESS){
            std::cout << ft_info.IPOC << "\t" << int(ft_info.code) << "\t" << ft_info.code_info;
            std::cout << "[\t";
            for (int j = 0; j < 6; j++)
            {
                std::cout << ft_data[j];
                std::cout << "\t";
            }
            std::cout << "]" << std::endl;
            Sleep(100);
        }
        else{
            std::cout << ft_rs485_handle <<" hps_ft_getData ERROR!"<< std::endl;
            std::cout << ft_info.IPOC << "\t" << int(ft_info.code) << "\t" << ft_info.code_info<< std::endl;
            break;
        }
    }

    if(hps_ft_uninitial(ft_rs485_handle)!=HPS_FT_SUCCESS){
        std::cout << ft_rs485_handle <<" hps_ft_uninitial ERROR!"<< std::endl;
    }

    hps_ft_deleteHandle(&ft_rs485_handle);
}

void test_ethernet_udp()
{
    double ft_data[6];
    hps_ft_info ft_info;
    HPS_FT_HANDLE ft_ethernet_udp_handle;
    ft_ethernet_udp_handle=hps_ft_createHandle(HpsSensorCommEnum::EtherNet_UDP);

    if(hps_ft_initial(ft_ethernet_udp_handle,8080,"192.168.5.99")!=HPS_FT_SUCCESS){
        std::cout << ft_ethernet_udp_handle <<" hps_ft_initial ERROR!"<< std::endl;
    }

    if(hps_ft_setLowPassFilter(ft_ethernet_udp_handle,0)!=HPS_FT_SUCCESS){
        std::cout << ft_ethernet_udp_handle <<" hps_ft_setLowPassFilter ERROR!"<< std::endl;
    }

    if(hps_ft_zero(ft_ethernet_udp_handle)!=HPS_FT_SUCCESS){
        std::cout << ft_ethernet_udp_handle <<" hps_ft_zero ERROR!"<< std::endl;
    }

    for (size_t i = 0; i < 10; i++)
    {
        if(hps_ft_getData(ft_ethernet_udp_handle,ft_data, ft_info)==HPS_FT_SUCCESS){
            std::cout << ft_info.IPOC << "\t" << int(ft_info.code) << "\t" << ft_info.code_info;
            std::cout << "[\t";
            for (int j = 0; j < 6; j++)
            {
                std::cout << ft_data[j];
                std::cout << "\t";
            }
            std::cout << "]" << std::endl;
            Sleep(100);
        }
        else{
            std::cout << ft_ethernet_udp_handle <<" hps_ft_getData ERROR!"<< std::endl;
            std::cout << ft_info.IPOC << "\t" << int(ft_info.code) << "\t" << ft_info.code_info<< std::endl;
            break;
        }
    }

//    if(hps_ft_uninitial(ft_ethernet_udp_handle)!=HPS_FT_SUCCESS){
//        std::cout << ft_ethernet_udp_handle <<" hps_ft_uninitial ERROR!"<< std::endl;
//    }

    hps_ft_deleteHandle(&ft_ethernet_udp_handle);
}
//----------------------------------------------------

void handle_test() {

    double ft_data[6];
    hps_ft_info ft_info;
    HPS_FT_HANDLE ft_ethernet_handle;
    uint8_t axis[6] = {0,0,0,0,0,0};            //alarm axis
    double value[6] = { 5,5,5,0.5,0.5,0.5 };    //alarm value
    int sum = 0;
    int failTime = 0;
    int successTime = 0;
//    int getDataFailTime = 0;
//    int getDataSuccessTime = 0;
    while (1){


        if ((ft_ethernet_handle = hps_ft_createHandle(HpsSensorCommEnum::EtherNet_UDP)) == -1){
            std::cout << "hps_ft_createHandle fail" << std::endl;
        }else{
            std::cout << "hps_ft_createHandle success" << std::endl;
        }
        if (hps_ft_initial(ft_ethernet_handle, 8080, "192.168.5.100") == HPS_FT_SUCCESS) {
            std::cout << "initial success" << std::endl;
            std::cout << ft_info.IPOC << "\t" << int(ft_info.code) << "\t" << ft_info.code_info;
            successTime++;
        }
        else
        {
            std::cout << ft_ethernet_handle << " hps_ft_initial ERROR!" << std::endl;
            std::cout << ft_info.IPOC << "\t" << int(ft_info.code) << "\t" << ft_info.code_info;
            failTime++;
        }

//        for (int i = 0; i < 10; ++i){
        //Set filtering
        if (hps_ft_setLowPassFilter(ft_ethernet_handle, 0) != HPS_FT_SUCCESS) {
            std::cout << ft_ethernet_handle << " hps_ft_setLowPassFilter ERROR!" << std::endl;
        }else {
            std::cout << ft_ethernet_handle << " hps_ft_setLowPassFilter SUCCESS!" << std::endl;
        }

        //Sensor zero clearing
        if (hps_ft_zero(ft_ethernet_handle) != HPS_FT_SUCCESS) {
            std::cout << ft_ethernet_handle << " hps_ft_zero ERROR!" << std::endl;
        }else {
            std::cout << ft_ethernet_handle << " hps_ft_zero SUCCESS!" << std::endl;
        }

        if (hps_ft_setAlarmThresholdValue(ft_ethernet_handle, value) != HPS_FT_SUCCESS) {
            std::cout << ft_ethernet_handle << " hps_ft_setAlarmThresholdValue ERROR!" << std::endl;
        }else {
            std::cout << ft_ethernet_handle << " hps_ft_setAlarmThresholdValue SUCCESS!" << std::endl;
        }
        if (hps_ft_setAlarm(ft_ethernet_handle,true) != HPS_FT_SUCCESS) {
            std::cout << ft_ethernet_handle << " hps_ft_setAlarm ERROR!" << std::endl;
        }else {
            std::cout << ft_ethernet_handle << " hps_ft_setAlarm SUCCESS!" << std::endl;
        }
        if (hps_ft_clearAlarmSignal(ft_ethernet_handle) != HPS_FT_SUCCESS) {
            std::cout << ft_ethernet_handle << " hps_ft_clearAlarmSignal ERROR!" << std::endl;
        }else {
            std::cout << ft_ethernet_handle << " hps_ft_clearAlarmSignal SUCCESS!" << std::endl;
        }
        for (int i = 0; i < 1000; ++i){
//        while (getDataFailTime <= 10)
//        {

                //Obtain sensor data and status information
            if (hps_ft_getData(ft_ethernet_handle, ft_data, ft_info) == HPS_FT_SUCCESS) {

                std::cout << ft_info.IPOC << "\t" << int(ft_info.code) << "\t" << ft_info.code_info;
                std::cout << "[\t";
                for (int j = 0; j < 6; j++)
                {
                   std::cout << ft_data[j];
                   std::cout << "\t";
                }
                std::cout << "]" << std::endl;

                Sleep(10);
                hps_ft_getAlarmAxis(ft_ethernet_handle, axis);
                for (size_t i = 0; i < sizeof(axis); ++i) {
                    //std::cout << i << static_cast<int>(axis[i]);
                    //std::cout << "\t";
                    printf("%x\t",axis[i]);

                }
                std::cout << "\n" << std::endl;
                hps_ft_clearAlarmSignal(ft_ethernet_handle);

            }
            else {
                std::cout << ft_ethernet_handle << " hps_ft_getData ERROR!" << std::endl;
                std::cout << ft_info.IPOC << "\t" << int(ft_info.code) << "\t" << ft_info.code_info << std::endl;
//                break;
            }
        }

        hps_ft_setStopSample(ft_ethernet_handle);


//        ft_ethernet_handle = 0;
        sum++;
        std::cout << "loop times:" << sum << std::endl;
        std::cout << "Success times:" << successTime << std::endl;
        std::cout << "Fail times:" << failTime << std::endl;
//        sleep(1);

//    }
        if (hps_ft_uninitial(ft_ethernet_handle) != HPS_FT_SUCCESS) {
            std::cout << ft_ethernet_handle << " hps_ft_uninitial ERROR!" << std::endl;
        }else {
            std::cout << ft_ethernet_handle << " hps_ft_uninitial SUCCESS!" << std::endl;
        }
        if (hps_ft_deleteHandle(&ft_ethernet_handle) != HPS_FT_SUCCESS) {
            std::cout << ft_ethernet_handle << " hps_ft_deleteHandle ERROR!" << std::endl;
        }else {
            std::cout << ft_ethernet_handle << " hps_ft_deleteHandle SUCCESS!" << std::endl;
        }
        }
    }


    //----------------------------------------------------

int main()
{
    std::cout << "Hello World!" << std::endl;

    //test_handle();

    test_rs485();

    // test_ethernet_udp();

    // handle_test();




    std::cout << "Hello World End!" << std::endl;
    return 0;
}


