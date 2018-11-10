/***********************************************************************
** Copyright (C) 2011 LP-Research
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

#ifndef DEVICE_LIST_ITEM
#define DEVICE_LIST_ITEM

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ImuData.h"

// Item for automatic device discovery
class DeviceListItem {
public:
    // Constructor
    DeviceListItem(const char *deviceId, int deviceType) :
        deviceType(deviceType) {
#ifdef _WIN32
        strcpy_s(this->deviceId, deviceId);
#else
        strcpy(this->deviceId, deviceId);
#endif

    }

    // Constructor
    DeviceListItem(void)
    {
    }

    // Device ID
    char deviceId[64];

    // Device type
    int deviceType;
};

// Contains all discovered devices
class LpmsDeviceList {
public:
    // Constructor
    LpmsDeviceList(void)
    {
        nDevices = 0;
    }

    // Adds a device
    void push_back(DeviceListItem i)
    {
        device[nDevices] = i;
        ++nDevices;
    }

    // Clears device list
    void clear(void)
    {
        nDevices = 0;
    }

    // Retrieves ID of device
    char *getDeviceId(int i) {
        return device[i].deviceId;
    }

    // Retrieves device type
    int getDeviceType(int i) {
        return device[i].deviceType;
    }

    // Writes devices to file
    void writeToFile(char *fn)
    {
        FILE *f;
#ifdef _WIN32
        fopen_s(&f, fn, "w+");
#else
        f = fopen(fn, "w+");
#endif

        if (f == NULL) return;

        for (int i = 0; i < nDevices; ++i) {
            fprintf(f, "%s %d\n", device[i].deviceId, device[i].deviceType);
        }

        fclose(f);
    }

    // Reads devices from file
    void readFromFile(char *fn)
    {
        FILE *f;
#ifdef _WIN32
        fopen_s(&f, fn, "r");
#else
        f = fopen(fn, "r");
#endif

        if (f == NULL) return;

        nDevices = 0;
        while (feof(f) == 0) {
            fprintf(f, "%s %d\n", device[nDevices].deviceId, device[nDevices].deviceType);
            ++nDevices;
        }

        fclose(f);
    }

public:
    DeviceListItem device[64];
    int nDevices;
};

#endif
