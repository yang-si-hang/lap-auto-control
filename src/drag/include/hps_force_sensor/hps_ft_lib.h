#ifndef HPS_FT_LIB_H
#define HPS_FT_LIB_H
#include <stdint.h>
#define _LINUX_
#define HPS_FT_LIB_LIBRARY
#ifdef _LINUX_
    #ifdef HPS_FT_LIB_LIBRARY
        #define HPS_FT_API __attribute__((visibility("default")))
    #else
        #define HPS_FT_API
    #endif
#else
    #ifdef HPS_FT_LIB_LIBRARY
        #define HPS_FT_API __declspec(dllexport)
    #else
        #define HPS_FT_API __declspec(dllimport)
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
#define HPS_FT_NULL -1
#define HPS_FT_FAIL 0
#define HPS_FT_SUCCESS 1
#define HPS_FT_SDK_VERSION_ "1.0.8"

typedef int HPS_FT_HANDLE;

#ifndef HPS_SENSOR_STRUCT
#define HPS_SENSOR_STRUCT
//Enumeration of sensor status codes
enum HpsInfoCode {
    FT_N0RMAL = 10000,
    ETHERNET_NO_SENSOR_ERROR = 11000,
    FT_GET_MATRIX_ERROR = 11001,
    FT_TEMP_COF_ERROR = 11002,
    FT_ADC_GAIN_ERROR = 11003,
    FT_ADC_NUM_ERROR = 11004,
    FT_ZERO_RESET_ERROR = 11005,
    FT_SET_DAC_ERROR = 11006,
    FT_NULL_MATRIX_ERROR = 11007,
    FT_DATA_ERROR = 11008,
    FT_ATTITUDE_SENSOR_INIT_ERROR = 11009,
    FT_ATTITUDE_SENSOR_DATA_ERROR = 11010,
    FT_OVERLOAD_ERROR = 11011,
    FT_NO_REFERENCE_VOLTAGE_ERROR = 11012,
    FT_NO_CROSSTALK_MATRIX_ERROR = 11013,

    //Communication status
    COMM_N0RMAL = 20000,
    COMM_CLOSE = 20001,
    COMM_INIT = 20002,
    COMM_IP_ERROR = 21000,
    COMM_PORT_ERROR = 21001,
    COMM_TIMEOUT_ERROR = 21002,
    COMM_CMDRETURN_ERROR = 21003,
    COMM_THREAD_OPEN_ERROR = 21004
};

//Enumeration of sensor communication protocol types
enum HpsSensorCommEnum {
    RS485,
    EtherNet_UDP,
    EtherNet_TCP
};

//Sensor information structure
typedef struct hps_ft_info_t
{
    HpsInfoCode code;       //sensor status code
    int IPOC;               //sensor data frame
    char code_info[256];    //status message
}hps_ft_info;

typedef struct hps_ft_deviceModeInfo_t
{
    uint8_t model;      //Model number
    uint8_t range;      //Range label
    uint16_t id;        //Unique id of the device
    uint8_t year;       //year
    uint8_t month;      //month
    uint8_t date;       //date
}hps_ft_deviceModeInfo;

typedef struct hps_ft_ip_host_t
{
    int host;
    char ip[256];
}hps_ft_ip_host;

/*
*f_par.kalman_K = 20;
*f_par.kalman_threshold = 3000;
*f_par.num_check = 3;
*t_par.kalman_K = 20;
*t_par.kalman_threshold = 2;
*t_par.num_check = 3;
*/
typedef struct KalmanFilterPara_t
{
    uint8_t kalman_K;
    int32_t kalman_threshold;
    uint32_t num_check;
}KalmanFilterPara;

typedef void (*FTCallback)(const int handle,const double[6],const hps_ft_info);
#endif



/*
*Create object
* You must create objects before you can interact
*/
HPS_FT_API HPS_FT_HANDLE  hps_ft_createHandle(HpsSensorCommEnum comm);

/*
*Destroy object
* You must disconnect the object before destroying it
*/
HPS_FT_API int  hps_ft_deleteHandle(HPS_FT_HANDLE *handle);



//General instruction
/*
*Connect the sensor and initialize
* The sensor object needs to be created first
*/
HPS_FT_API int  hps_ft_initial(HPS_FT_HANDLE handle, int port, const char *ip);

/*
*Disconnect the sensor and de-initialize
* Execute this function before the object is destroyed
*/
HPS_FT_API int  hps_ft_uninitial(HPS_FT_HANDLE handle);

/*
*Sensor zero clearing
*/
HPS_FT_API int  hps_ft_zero(HPS_FT_HANDLE handle);

/*
*Obtain real-time sensor data
*/
HPS_FT_API int  hps_ft_getData2(HPS_FT_HANDLE handle, double m_ftData[6]);

/*
*Obtain real-time sensor data and status information
*/
HPS_FT_API int  hps_ft_getData(HPS_FT_HANDLE handle, double m_ftData[6], hps_ft_info &info);

/*
 * Register callback functions to get sensor data and status
 * Full frame data can be obtained through the callback function
 * Processing time-consuming tasks will affect the frame rate
*/
HPS_FT_API int  hps_ft_registerCallback(HPS_FT_HANDLE handle, FTCallback callback);

/*
 * Destroy the registered callback function
*/
HPS_FT_API int  hps_ft_dellCallback(HPS_FT_HANDLE handle);

/*
*Get the sensor range
*/
HPS_FT_API int  hps_ft_getRange(HPS_FT_HANDLE handle, double range[]);

/*
*Obtain sensor status information
*/
HPS_FT_API int  hps_ft_getInfo(HPS_FT_HANDLE handle, hps_ft_info &info);

/*
*Get the sensor IPHost
*/
HPS_FT_API int  hps_ft_getIPHost(HPS_FT_HANDLE handle, hps_ft_ip_host &iphost);

/*
*Get the sensor DeviceModeInfo
*/
HPS_FT_API int  hps_ft_getDeviceModeInfo(HPS_FT_HANDLE handle, hps_ft_deviceModeInfo &deviceModeInfo);

HPS_FT_API int  hps_ft_getDeviceModeInfo2(HPS_FT_HANDLE handle, int *deviceModeInfo);

/*
*The sensor stops the data flow transmission mode
* After the data stream transmission mode is disabled,
* the sensor works in single mode and sends a command to obtain data once.
*/
HPS_FT_API int  hps_ft_setStopSample(HPS_FT_HANDLE handle);

/*
*The sensor starts the data stream transmission mode
* The data stream transmission mode is enabled by default,
* and data is continuously collected at the highest frame rate.
*/
HPS_FT_API int  hps_ft_setStartSample(HPS_FT_HANDLE handle);

/*
*Set the low-pass filtering level
* Filtering range (0-6). 0 indicates that the filtering function is disabled.
* The frame rate of the sensor is halved for each filtering level
*/
HPS_FT_API int  hps_ft_setLowPassFilter(HPS_FT_HANDLE handle, uint8_t range);

/*
*Save user Settings
* After the device is powered off and restarted, the parameters are still valid.
* This command takes a long time. You are advised to wait at least 500ms.
*/
HPS_FT_API int  hps_ft_saveUserSetting(HPS_FT_HANDLE handle);



//Ethernet dependent instruction
/*
*Sensor center offset (try not to use)
*/
HPS_FT_API int  hps_ft_setToolTransform(HPS_FT_HANDLE handle, double ToolTransform[6]);



//Filter instruction
/*
*IIR filtering is mutually exclusive with FTR
*/
HPS_FT_API int  hps_ft_setIIRFilter(HPS_FT_HANDLE handle, bool enable);

/*
*FIR filtering is mutually exclusive with ITR
*/
HPS_FT_API int  hps_ft_setFIRFilter(HPS_FT_HANDLE handle, bool enable);

/*
*Set the median filter
* Filtering range (0-64). 0 indicates that the filtering function is disabled.
*/
HPS_FT_API int  hps_ft_setMedianFilter(HPS_FT_HANDLE handle, uint8_t range);

/*
*Set the moving average filter
* Filtering range (0-64). 0 indicates that the filtering function is disabled.
*/
HPS_FT_API int  hps_ft_setSmoothAverFilter(HPS_FT_HANDLE handle, uint8_t range);

/*
*Set the Kalman filter
*/
HPS_FT_API int  hps_ft_setKalmanFilter(HPS_FT_HANDLE handle, bool isendble, KalmanFilterPara f_para, KalmanFilterPara t_para);



//Ethernet setup instruction
/*
*Set the IP address of the switching box
*/
HPS_FT_API int  hps_ft_setNetIP(HPS_FT_HANDLE handle, const char *ip);

/*
*Set the mask of the switching box
*/
HPS_FT_API int  hps_ft_setNetMask(HPS_FT_HANDLE handle, const char *mask);

/*
*Set the gateway of the Ethernet conversion box
*/
HPS_FT_API int  hps_ft_setNetGateway(HPS_FT_HANDLE handle, const char *gateway);

/*
*Set the port number of the Ethernet conversion box
*/
HPS_FT_API int  hps_ft_setNetPortNumber(HPS_FT_HANDLE handle, uint16_t portNumber);




//IO alarm instruction
/*
*Activate the IO alarm function
*/
HPS_FT_API int  hps_ft_setAlarmSignal(HPS_FT_HANDLE handle);

/*
*Disable the IO alarm function
*/
HPS_FT_API int  hps_ft_clearAlarmSignal(HPS_FT_HANDLE handle);

/*
*Manually trigger the IO alarm
*/
HPS_FT_API int  hps_ft_setAlarm(HPS_FT_HANDLE handle, bool enable);//enable

/*
*Get IO alarm status of each axis
*/
HPS_FT_API int  hps_ft_getAlarmAxis(HPS_FT_HANDLE handle, uint8_t axis[6]);

/*
*Set the IO alarm to normally on by default
*/
HPS_FT_API int  hps_ft_setAlarmNormalOpen(HPS_FT_HANDLE handle, bool enable);

/*
*Set the alarm threshold
*/
HPS_FT_API int  hps_ft_setAlarmThresholdValue(HPS_FT_HANDLE handle, double value[6]);//设置报警阈值

#ifdef __cplusplus
}
#endif
#endif // HPS_FT_LIB_H
