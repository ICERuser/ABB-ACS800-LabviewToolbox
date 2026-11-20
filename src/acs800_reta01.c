#include "acs800_reta01.h"

#include <windows.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#ifndef ACS800_CRC_INITIAL
#define ACS800_CRC_INITIAL 0xFFFFu
#endif

static HANDLE s_port = INVALID_HANDLE_VALUE;
static char s_error[512] = "No error";
static unsigned int s_timeoutMs = 1000;

static void set_error(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(s_error, sizeof(s_error), fmt, args);
    va_end(args);
}

static unsigned short modbus_crc16(const unsigned char *data, size_t length) {
    unsigned short crc = (unsigned short)ACS800_CRC_INITIAL;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? (unsigned short)((crc >> 1) ^ 0xA001u)
                             : (unsigned short)(crc >> 1);
        }
    }
    return crc;
}

static int write_all(const unsigned char *data, DWORD length) {
    DWORD written = 0;
    if (!WriteFile(s_port, data, length, &written, NULL) || written != length) {
        set_error("WriteFile failed (error %lu)", GetLastError());
        return 0;
    }
    return 1;
}

static int read_exact(unsigned char *data, DWORD length) {
    DWORD received = 0;
    if (!ReadFile(s_port, data, length, &received, NULL) || received != length) {
        set_error("ReadFile failed or timed out (error %lu, received %lu)",
                  GetLastError(), received);
        return 0;
    }
    return 1;
}

ACS800_API int ACS800_Open(const char *comPort, unsigned int baud,
                           unsigned int dataBits, char parity,
                           unsigned int stopBits, unsigned int timeoutMs) {
    char deviceName[128];
    DCB dcb;
    COMMTIMEOUTS timeout;

    if (comPort == NULL || comPort[0] == '\0') {
        set_error("COM port is required");
        return 0;
    }

    if (_snprintf(deviceName, sizeof(deviceName), "\\\\.\\%s", comPort) < 0) {
        set_error("COM port name is too long");
        return 0;
    }

    ACS800_Close();

    s_port = CreateFileA(deviceName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                         OPEN_EXISTING, 0, NULL);
    if (s_port == INVALID_HANDLE_VALUE) {
        set_error("Could not open %s (error %lu)", comPort, GetLastError());
        return 0;
    }

    ZeroMemory(&dcb, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(s_port, &dcb)) {
        set_error("GetCommState failed (error %lu)", GetLastError());
        ACS800_Close();
        return 0;
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = (BYTE)dataBits;
    dcb.Parity = (parity == 'E' || parity == 'e') ? EVENPARITY
                : (parity == 'O' || parity == 'o') ? ODDPARITY
                : NOPARITY;
    dcb.StopBits = (stopBits == 1) ? ONESTOPBIT
                 : (stopBits == 2) ? TWOSTOPBITS
                 : ONE5STOPBITS;

    if (!SetCommState(s_port, &dcb)) {
        set_error("SetCommState failed (error %lu)", GetLastError());
        ACS800_Close();
        return 0;
    }

    ZeroMemory(&timeout, sizeof(timeout));
    timeout.ReadIntervalTimeout = timeoutMs;
    timeout.ReadTotalTimeoutConstant = timeoutMs;
    timeout.ReadTotalTimeoutMultiplier = 0;
    timeout.WriteTotalTimeoutConstant = timeoutMs;
    s_timeoutMs = timeoutMs;

    if (!SetCommTimeouts(s_port, &timeout)) {
        set_error("SetCommTimeouts failed (error %lu)", GetLastError());
        ACS800_Close();
        return 0;
    }

    PurgeComm(s_port, PURGE_RXCLEAR | PURGE_TXCLEAR);
    set_error("No error");
    return 1;
}

ACS800_API int ACS800_Close(void) {
    if (s_port != INVALID_HANDLE_VALUE) {
        CloseHandle(s_port);
        s_port = INVALID_HANDLE_VALUE;
    }
    return 1;
}

ACS800_API int ACS800_ReadHoldingRegisters(unsigned char slave,
                                           unsigned short startAddress,
                                           unsigned short quantity,
                                           unsigned short *values,
                                           unsigned int valuesCapacity) {
    unsigned char request[8];
    unsigned char response[256];
    unsigned short crc;

    if (s_port == INVALID_HANDLE_VALUE) {
        set_error("Port is not open");
        return 0;
    }
    if (slave == 0 || quantity == 0 || quantity > 125) {
        set_error("Invalid slave or quantity");
        return 0;
    }
    if (values == NULL || valuesCapacity < quantity) {
        set_error("Output buffer is NULL or too small");
        return 0;
    }

    request[0] = slave;
    request[1] = 0x03;
    request[2] = (unsigned char)(startAddress >> 8);
    request[3] = (unsigned char)(startAddress & 0xFF);
    request[4] = (unsigned char)(quantity >> 8);
    request[5] = (unsigned char)(quantity & 0xFF);

    crc = modbus_crc16(request, 6);
    request[6] = (unsigned char)(crc & 0xFF);
    request[7] = (unsigned char)(crc >> 8);

    PurgeComm(s_port, PURGE_RXCLEAR);
    if (!write_all(request, sizeof(request))) return 0;

    if (!read_exact(response, 3)) return 0;

    if (response[0] != slave) {
        set_error("Unexpected slave address in response");
        return 0;
    }
    if (response[1] == 0x83) {
        if (!read_exact(response + 3, 3)) return 0;
        set_error("Modbus exception %u", (unsigned int)response[2]);
        return 0;
    }
    if (response[1] != 0x03) {
        set_error("Unexpected Modbus function code 0x%02X", response[1]);
        return 0;
    }

    DWORD byteCount = response[2];
    if (byteCount != quantity * 2u || byteCount > 250) {
        set_error("Invalid byte count %lu", byteCount);
        return 0;
    }

    if (!read_exact(response + 3, byteCount + 2u)) return 0;

    unsigned char *frame = response;
    DWORD frameLength = 3u + byteCount + 2u;
    unsigned short receivedCrc = (unsigned short)frame[frameLength - 2]
                               | ((unsigned short)frame[frameLength - 1] << 8);
    unsigned short calculatedCrc = modbus_crc16(frame, frameLength - 2);
    if (receivedCrc != calculatedCrc) {
        set_error("Modbus CRC mismatch");
        return 0;
    }

    for (unsigned short i = 0; i < quantity; ++i) {
        values[i] = (unsigned short)((unsigned short)frame[3u + i * 2u] << 8)
                  | (unsigned short)frame[4u + i * 2u];
    }

    set_error("No error");
    return 1;
}

ACS800_API const char *ACS800_LastError(void) {
    return s_error;
}
