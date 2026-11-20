#ifndef ACS800_RETA01_H
#define ACS800_RETA01_H

#include <stdint.h>

#ifdef _WIN32
  #ifdef ACS800_DLL_EXPORTS
    #define ACS800_API __declspec(dllexport)
  #else
    #define ACS800_API __declspec(dllimport)
  #endif
#else
  #define ACS800_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

ACS800_API int ACS800_Open(const char *comPort, unsigned int baud,
                           unsigned int dataBits, char parity,
                           unsigned int stopBits, unsigned int timeoutMs);

ACS800_API int ACS800_Close(void);

ACS800_API int ACS800_ReadHoldingRegisters(unsigned char slave,
                                           unsigned short startAddress,
                                           unsigned short quantity,
                                           unsigned short *values,
                                           unsigned int valuesCapacity);

ACS800_API const char *ACS800_LastError(void);

#ifdef __cplusplus
}
#endif

#endif
