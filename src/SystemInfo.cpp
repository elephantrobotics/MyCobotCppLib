#include "SystemInfo.hpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ARR_SIZE 16

namespace rc {

double SystemInfo::GetCpuTemperature()
{
    char buffer[1024];
#if defined(__linux__)
    FILE *fpp = popen("sensors | sed -rn \'s/.*Core [0-9.]:\\s+.([0-9.]+).\\1/p'", "r");
#else
    FILE *fpp = _popen("sensors | sed -rn \'s/.*Core [0-9.]:\\s+.([0-9.]+).\\1/p'", "r");
#endif
    if (fpp == nullptr) {
        return ImpossibleTemperature;
    }
    int count = 0;
    int char_count = 0;
    char *temp_arr[ARR_SIZE] = {NULL};
    while (fgets(buffer, sizeof(buffer), fpp) != NULL) {
        while (buffer[char_count] != '\n')
            ++char_count;
        temp_arr[count] = static_cast<char*>(malloc(char_count));
        if (temp_arr[count] == nullptr) {
            continue;
        }
        strncpy(temp_arr[count], buffer, char_count);
        *(temp_arr[count] + char_count) = '\0';
        ++count;
    }
#if defined(__linux__)
    pclose(fpp);
#else
    _pclose(fpp);
#endif
    if (count == 0)
        return ImpossibleTemperature;
    int i = 0;
    double maximum = atof(temp_arr[i]);
    for (i = 1; i < count; ++i) {
        if (atof(temp_arr[i]) >= maximum)
            maximum = atof(temp_arr[i]);
    }
    for (i = 0; i < count; ++i) {
        free(temp_arr[i]);
    }
    return 0;
}

}
