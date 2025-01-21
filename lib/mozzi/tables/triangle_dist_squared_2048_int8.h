#ifndef TRIANGLE_DIST_SQUARED_2048_H_
#define TRIANGLE_DIST_SQUARED_2048_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <avr/pgmspace.h>

/* triangle wave with distortion from Audacity
*/

#define TRIANGLE_DIST_SQUARED_2048_NUM_CELLS 2048
#define TRIANGLE_DIST_SQUARED_2048_SAMPLERATE 2048

const int8_t __attribute__((section(".progmem.data"))) TRIANGLE_DIST_SQUARED_2048_DATA []  =
        {
                16, 16, 16, 16, 15, 16,
                16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 15,
                15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 14,
                14, 13, 13, 13, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 13,
                13, 12, 12, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11,
                12, 12, 11, 11, 11, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
                11, 11, 11, 11, 11, 11, 10, 11, 11, 10, 10, 11, 11, 10, 10, 10, 10, 10, 10, 10,
                10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9,
                9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 9, 9, 8, 8, 9, 9, 8, 8, 8, 8,
                8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 8, 8, 7, 7, 8,
                8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6,
                6, 7, 7, 6, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                6, 6, 6, 6, 6, 5, 6, 6, 6, 5, 6, 6, 5, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
                5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 5, 5, 5, 4,
                5, 5, 4, 5, 5, 5, 4, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 4, 4, 4, 4, 4, 4,
                3, 3, 4, 4, 3, 4, 4, 3, 3, 4, 4, 3, 3, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 3, 3, 2, 2, 3, 3, 2, 2, 3, 3, 2, 3, 3, 2, 2,
                3, 3, 2, 2, 3, 3, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0,
                0, 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -6, -6, -6, -7, -7,
                -7, -7, -8, -9, -9, -9, -10, -11, -11, -11, -12, -13, -13, -13, -14, -15, -15,
                -15, -17, -17, -17, -18, -19, -20, -20, -20, -22, -22, -23, -24, -25, -25, -25,
                -27, -28, -28, -28, -30, -31, -31, -32, -34, -35, -35, -35, -37, -38, -38, -38,
                -39, -41, -42, -42, -42, -45, -46, -46, -46, -49, -50, -50, -53, -54, -54, -54,
                -57, -58, -58, -58, -58, -61, -62, -62, -65, -66, -65, -65, -65, -68, -69, -69,
                -69, -71, -72, -72, -74, -75, -75, -74, -76, -77, -77, -76, -78, -79, -78, -78,
                -79, -79, -79, -79, -78, -79, -79, -79, -79, -79, -78, -78, -78, -78, -77, -77,
                -76, -76, -75, -75, -74, -73, -73, -72, -72, -71, -70, -70, -69, -67, -66, -66,
                -66, -64, -62, -62, -62, -60, -58, -59, -58, -56, -54, -54, -54, -52, -50, -50,
                -47, -46, -46, -46, -46, -43, -42, -42, -42, -39, -38, -38, -35, -34, -34, -34,
                -31, -30, -30, -30, -27, -26, -27, -26, -24, -23, -23, -23, -23, -21, -19, -20,
                -20, -17, -16, -17, -17, -14, -13, -14, -11, -10, -11, -11, -11, -9, -8, -8, -8,
                -6, -5, -6, -6, -4, -3, -4, -4, -2, -1, -1, 1, 1, 1, 1, 0, 2, 3, 2, 2, 4, 5, 4,
                4, 6, 6, 6, 7, 8, 7, 7, 7, 8, 9, 8, 10, 11, 10, 10, 9, 11, 11, 11, 11, 12, 12,
                12, 13, 14, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 14, 15, 16, 15, 17, 17, 16,
                16, 16, 17, 17, 17, 16, 17, 18, 17, 17, 18, 18, 18, 17, 18, 19, 18, 19, 19, 19,
                19, 19, 20, 19, 19, 19, 19, 20, 19, 19, 20, 20, 20, 20, 21, 20, 20, 21, 21, 20,
                20, 20, 20, 21, 20, 20, 21, 21, 20, 21, 21, 21, 20, 21, 21, 21, 21, 21, 21, 21,
                21, 20, 21, 21, 21, 20, 21, 21, 21, 21, 21, 21, 21, 20, 21, 21, 21, 21, 21, 21,
                21, 20, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 20, 21, 21, 20, 20, 21, 21,
                20, 20, 21, 21, 20, 20, 20, 21, 20, 20, 20, 21, 20, 21, 21, 20, 20, 20, 20, 20,
                20, 20, 20, 20, 20, 19, 20, 20, 20, 19, 20, 20, 19, 19, 20, 20, 19, 19, 19, 20,
                19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 18, 19, 19, 18, 19, 19, 19,
                18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 17, 18, 18, 18, 17, 18, 18,
                17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
                16, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
                15, 15, 15, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
                14, 15, 15, 14, 14, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 14, 14,
                13, 13, 14, 14, 13, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
                13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
                12, 11, 12, 12, 11, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
                11, 11, 11, 11, 11, 10, 11, 11, 10, 10, 11, 11, 10, 11, 11, 10, 10, 10, 10, 10,
                10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 10, 10, 9, 9, 9, 9, 9,
                10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8,
                8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 8,
                8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
                7, 7, 7, 7, 7, 7, 7, 6, 7, 7, 6, 6, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 6, 6, 6, 5, 6, 6, 5, 5, 6, 6, 5, 6, 6, 5, 5, 5, 5,
                5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
                5, 5, 5, 4, 5, 5, 4, 4, 5, 5, 4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                4, 4, 4, 4, 4, 3, 4, 4, 3, 4, 4, 4, 3, 3, 3, 4, 3, 4, 4, 3, 3, 3, 4, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 3, 3, 2, 2, 3, 3, 2, 3, 3, 3, 2,
                3, 3, 3, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 1,
                1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -5, -6,
                -6, -6, -7, -7, -7, -8, -9, -9, -9, -9, -10, -11, -11, -11, -11, -13, -13, -13,
                -15, -15, -15, -16, -16, -17, -18, -18, -19, -20, -20, -20, -22, -23, -23, -23,
                -23, -25, -26, -26, -26, -28, -29, -29, -31, -32, -32, -32, -34, -36, -35, -36,
                -38, -39, -39, -39, -42, -43, -43, -43, -45, -47, -47, -47, -49, -51, -50, -51,
                -53, -55, -54, -55, -57, -59, -58, -59, -58, -61, -63, -62, -65, -67, -66, -66,
                -69, -70, -69, -69, -69, -72, -73, -72, -75, -75, -75, -75, -77, -77, -77, -77,
                -78, -79, -78, -78, -79, -79, -79, -79, -78, -79, -79, -78, -79, -79, -78, -78,
                -77, -77, -77, -76, -76, -75, -75, -74, -73, -72, -72, -72, -71, -70, -69, -69,
                -68, -67, -65, -65, -63, -62, -62, -62, -61, -59, -58, -58, -55, -54, -54, -53,
                -51, -49, -50, -49, -49, -46, -45, -45, -42, -41, -41, -41, -41, -38, -37, -37,
                -34, -33, -33, -33, -33, -30, -29, -29, -29, -27, -25, -26, -23, -22, -22, -22,
                -20, -18, -19, -19, -19, -17, -15, -16, -16, -14, -13, -13, -11, -10, -10, -10,
                -8, -7, -8, -8, -8, -6, -5, -5, -6, -3, -3, -3, -1, 0, -1, -1, 1, 2, 1, 1, 3, 4,
                3, 3, 4, 5, 5, 4, 6, 7, 6, 6, 8, 8, 8, 7, 7, 9, 9, 9, 10, 11, 10, 10, 10, 11,
                12, 11, 12, 13, 12, 12, 12, 13, 14, 13, 13, 14, 14, 14, 15, 16, 15, 15, 16, 16,
                16, 15, 15, 16, 17, 16, 16, 17, 17, 17, 16, 17, 18, 17, 17, 18, 18, 18, 19, 19,
                19, 18, 18, 19, 19, 19, 18, 19, 19, 19, 19, 19, 20, 19, 20, 20, 20, 20, 19, 20,
                20, 20, 21, 21, 20, 20, 21, 21, 20, 20, 20, 21, 21, 20, 20, 21, 21, 20, 20, 21,
                21, 21, 20, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 20, 21, 21, 21, 20, 21,
                21, 21, 21, 21, 21, 21, 20, 21, 21, 21, 20, 21, 21, 21, 21, 21, 21, 21, 20, 21,
                21, 20, 21, 21, 21, 20, 21, 21, 21, 20, 20, 21, 21, 20, 21, 21, 20, 20, 21, 21,
                20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 19, 20, 20, 20, 20, 20, 20, 19, 20, 20,
                20, 19, 19, 19, 19, 19, 19, 20, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 18, 19,
                19, 18, 19, 19, 18, 18, 19, 19, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
                18, 17, 17, 18, 18, 17, 18, 18, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
                17, 17, 17, 17, 17, 16, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
                16,
        };

#endif /* TRIANGLE_DIST_SQUARED_2048_H_ */
