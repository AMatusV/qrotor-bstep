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

#ifndef IMU_DATA
#define IMU_DATA

typedef struct _HeaveMotionModule {
    float yHeave;
} HeaveMotionModule;

typedef struct _GaitTrackingModule {
    float zGait;
    float zAmplitude;
    float yGait;
    float yAmplitude;
    float frequency;
    float velocity;
    float symmetry;
    int zDirection;
    int yDirection;
} GaitTrackingModule;

// Structure for data exchange inside the OpenMAT network.
typedef struct _ImuData {
    // The OpenMAT ID of the sensor that created this data structure.
    int openMatId;

    // Calibrated accelerometer sensor data.
    float a[3];

    // Calibrated gyroscope sensor data.
    float g[3];

    // Calibrated magnetometer sensor data.
    float b[3];

    // Angular velocity data.
    float w[3];

    // Euler angle data.
    float r[3];

    // Quaternion orientation data.
    float q[4];

    // Orientation data as rotation matrix without offset.
    float rotationM[9];

    // Orientation data as rotation matrix after zeroing.
    float rotOffsetM[9];

    // Raw accelerometer sensor data.
    float aRaw[3];

    // Raw gyroscope sensor data.
    float gRaw[3];

    // Raw magnetometer sensor data.
    float bRaw[3];

    // Barometric pressure.
    float pressure;

    // Index of the data frame.
    int frameCount;

    // Linear acceleration x, y and z.
    float linAcc[3];

    // Gyroscope temperature.
    float gTemp;

    // Altitude.
    float altitude;

    // Temperature.
    float temperature;

    // Sampling time of the data.
    double timeStamp;

    HeaveMotionModule hm;
    GaitTrackingModule gm;

} ImuData;

#endif