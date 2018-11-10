/***********************************************************************
** Copyright (C) 2012 LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** Redistribution and use in source and binary forms, with
** or without modification, are permitted provided that the
** following conditions are met:
**
** Redistributions of source code must retain the above copyright
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright
** notice, this list of conditions and the following disclaimer in
** the documentation and/or other materials provided with the
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#ifndef LPMS_SENSOR_I
#define LPMS_SENSOR_I

#ifdef _WIN32
#include "windows.h"
#endif
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "ImuData.h"
#include "LpmsDefinitions.h"
//#include "LpMatrix.h"

typedef void(*LpmsCallback)(ImuData d, const char* id);

class LpmsSensorI
{
public:
    /***********************************************************************
    ** CONSTRUCTORS / DESTRUCTORS
    ***********************************************************************/
    ~LpmsSensorI(void) {};

    /***********************************************************************
    ** POLL / UPDATE DATA FROM SENSORS
    ***********************************************************************/
    virtual void pollData(void) = 0;
    virtual void assertConnected(void) = 0;
    virtual void update(void) = 0;

    /***********************************************************************
    ** DIRECT GET / SET DEVICE PARAMETERS
    ***********************************************************************/
    virtual void setVerbose(bool v) = 0;
    virtual void getDeviceId(char *str) = 0;
    // virtual CalibrationData *getConfigurationData(void) = 0;
    virtual bool assertFwVersion(int d0, int d1, int d2) = 0;
    virtual bool hasNewFieldMap(void) = 0;
    virtual void setFps(float f) = 0;
    virtual float getFps(void) = 0;
    virtual void setSensorStatus(int s) = 0;
    virtual int getSensorStatus(void) = 0;
    virtual void setConnectionStatus(int s) = 0;
    virtual int getConnectionStatus(void) = 0;
    virtual void setCurrentData(ImuData d) = 0;
    virtual void setCallback(LpmsCallback cb) = 0;
    virtual ImuData getCurrentData(void) = 0;
    virtual int hasImuData(void) = 0;
    virtual void getCalibratedSensorData(float g[3], float a[3], float b[3]) = 0;
    virtual void getQuaternion(float q[4]) = 0;
    virtual void getEulerAngle(float r[3]) = 0;
    virtual void getRotationMatrix(float M[3][3]) = 0;
    virtual	void getGyroData(float g[3]) = 0;
    virtual void getAccelerometerData(float a[3]) = 0;
    virtual bool isRunning(void) = 0;
    virtual void pause(void) = 0;
    virtual void run(void) = 0;
    virtual void close(void) = 0;
    virtual void startCalibrateGyro(void) = 0;
    virtual void setOrientationOffset(int v) = 0;
    virtual void resetOrientationOffset(void) = 0;
    virtual void setOpenMatId(int id) = 0;
    virtual int getOpenMatId(void) = 0;
    virtual bool updateParameters(void) = 0;
    virtual bool setConfigurationPrm(int parameterIndex, int parameter) = 0;
    virtual bool setConfigurationPrm(int parameterIndex, int *parameter) = 0;
    virtual bool getConfigurationPrm(int parameterIndex, int* parameter) = 0;
    virtual bool getConfigurationPrm(int parameterIndex, char* parameter) = 0;
    virtual float getBatteryLevel(void) = 0;
    virtual float getBatteryVoltage(void) = 0;
    virtual int getChargingStatus(void) = 0;
    virtual std::string getDeviceName(void) = 0;
    virtual std::string getFirmwareInfo(void) = 0;
    virtual void startSync() = 0;
    virtual void stopSync() = 0;
    virtual void getGyroStaticBias() = 0;

    /***********************************************************************
    ** FIRMWARE / IAP
    ***********************************************************************/
    virtual bool uploadFirmware(const char *fn) = 0;
    virtual bool uploadIap(const char *fn) = 0;
    virtual int getUploadProgress(int *p) = 0;
    virtual void saveCalibrationData(void) = 0;
    //	virtual LpmsIoInterface *getIoInterface(void) = 0;
    virtual void measureAvgLatency(void) = 0;
    virtual void acquireFieldMap(void) = 0;
    virtual bool getPressure(float *p) = 0;
    virtual void getHardIronOffset(float v[3]) = 0;
    virtual void getSoftIronMatrix(float M[3][3], float *fieldRadius) = 0;
    virtual float getFieldNoise(void) = 0;
    virtual void getFieldMap(float fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3]) = 0;
    virtual void zeroFieldMap(void) = 0;
    virtual void resetToFactorySettings(void) = 0;
    virtual long getStreamFrequency(void) = 0;
    virtual void armTimestampReset(void) = 0;
    virtual void setUploadPageSize(int size) = 0;

    /***********************************************************************
    ** CALIBRATION
    ***********************************************************************/
    virtual void startPlanarMagCalibration(void) = 0;
    virtual void checkPlanarMagCal(float T) = 0;
    virtual void stopPlanarMagCalibration(void) = 0;
    virtual void startMagCalibration(void) = 0;
    virtual void checkMagCal(float T) = 0;
    virtual void stopMagCalibration(void) = 0;
    virtual void initMisalignCal(void) = 0;
    virtual void startGetMisalign(int i) = 0;
    virtual void checkMisalignCal(float T) = 0;
    virtual void calcMisalignMatrix(void) = 0;
    virtual void stopMisalignCal(void) = 0;
    virtual void saveCalibrationData(const char* fn) = 0;
    virtual bool loadCalibrationData(const char* fn) = 0;
    virtual void initGyrMisalignCal(void) = 0;
    virtual void startGetGyrMisalign(int i) = 0;
    virtual void checkGyrMisalignCal(float T) = 0;
    virtual void calcGyrMisalignMatrix(void) = 0;
    virtual void stopGyrMisalignCal(void) = 0;
    virtual void resetTimestamp(void) = 0;
    virtual void setTimestamp(float t) = 0;
    virtual void startAutoMagMisalignCal(void) = 0;
    virtual void checkAutoMagMisalignCal(float T) = 0;
    virtual void calcAutoMagMisalignCal(void) = 0;
    virtual void startMagReferenceCal(void) = 0;
    virtual void checkMagReferenceCal(float T) = 0;
    virtual void calcMagReferenceCal(void) = 0;
    virtual void initMagMisalignCal(void) = 0;
    virtual void startMagMisalignCal(int i) = 0;
    virtual void checkMagMisalignCal(float T) = 0;
    virtual void calcMagMisalignCal(void) = 0;

    //virtual LpVector3f getMisalignAData(int i) = 0;
    //virtual LpVector3f getGyrMisalignAData(int i) = 0;
    virtual bool isSamplingAcc() = 0;
    virtual bool isSamplingGyro() = 0;

    /***********************************************************************
    ** DATA RECORDING
    ***********************************************************************/
    virtual void startSaveData(std::ofstream *saveDataHandle) = 0;
    virtual void checkSaveData(void) = 0;
    virtual void stopSaveData(void) = 0;


    /***********************************************************************
    ** LPMSB2 Internal Flash Logging
    ***********************************************************************/
    virtual void startFlashLogging() = 0;
    virtual void stopFlashLogging() = 0;
    virtual void clearFlashLog() = 0;
    virtual void fullEraseFlash() = 0;
    virtual void getFlashLoggingStatus() = 0;
    virtual void getFlashMetaTableSize() = 0;
    virtual void getFlashMetaTable() = 0;
    virtual void getFlashLogSize() = 0;
    virtual bool getFlashLog(const char* fn) = 0;
    virtual void cancelGetFlashLog() = 0;
    virtual bool isDownloadingFlashLog() = 0;
    virtual int getDownloadFlashLogProgress(int *p) = 0;
    virtual std::string getErroMsg() = 0;
};

#ifdef _WIN32
#ifdef DLL_EXPORT
#define LPMS_API __declspec(dllexport)
#else
#define LPMS_API __declspec(dllimport)
#endif

extern "C" LPMS_API	LpmsSensorI* APIENTRY LpmsSensorFactory(int deviceType, const char *deviceId);
#endif

#endif

