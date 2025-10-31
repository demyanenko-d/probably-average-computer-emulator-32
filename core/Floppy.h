#pragma once

#include <cstdint>

inline void guessFloppyImageGeometry(uint32_t fdSize, bool &doubleSided, int &sectorsPerTrack)
{
    switch(fdSize / 1024)
    {
        case 160:
            doubleSided = false;
            sectorsPerTrack = 8;
            break;
        case 180:
            doubleSided = false;
            sectorsPerTrack = 9;
            break;
        case 360:
            doubleSided = true;
            sectorsPerTrack = 9;
            // could also be a single-sided 3.5-inch disk
            break;
        case 720: // 3.5 inch
            doubleSided = true;
            sectorsPerTrack = 9;
            break;
        case 1200:
            doubleSided = true;
            sectorsPerTrack = 15;
            break;
        case 1440: // 3.5 inch
            doubleSided = true;
            sectorsPerTrack = 18;
            break;
        default:
            doubleSided = false;
            sectorsPerTrack = 0;
            break;
    }
}