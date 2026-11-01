#ifndef IMAGE_EIGHT_H
#define IMAGE_EIGHT_H

#include "zf_common_headfile.h"
#include "Camera.h"

#define EIGHT_NEIGHBOR_ENABLE   1

#define EIGHT_IMAGE_H           MT9V03X_H
#define EIGHT_IMAGE_W           MT9V03X_W
#define EIGHT_MAX_POINTS        360

#define EIGHT_WHITE_PIXEL       255
#define EIGHT_BLACK_PIXEL       0
#define EIGHT_BORDER_MIN        1
#define EIGHT_BORDER_MAX        (EIGHT_IMAGE_W - 2)

typedef struct
{
    int16 left[EIGHT_IMAGE_H];
    int16 right[EIGHT_IMAGE_H];
    int16 center[EIGHT_IMAGE_H];

    int16 points_l[EIGHT_MAX_POINTS][2];
    int16 points_r[EIGHT_MAX_POINTS][2];

    uint16 points_l_num;
    uint16 points_r_num;

    uint8 highest;
    uint8 valid;
} EightNeighborFrame;

extern EightNeighborFrame eight_frame;

extern int16 eight_center_crop[LCDH];
extern uint8 eight_center_valid[LCDH];

extern int eight_diff_rows;
extern int eight_diff_average;
extern int eight_diff_max;

void EightNeighbor_Process(const uint8 image[EIGHT_IMAGE_H][EIGHT_IMAGE_W]);
void EightNeighbor_ToLegacy(void);
void EightNeighbor_Compare(void);
void EightNeighbor_PublishEdgesToImageDeal(void);

#endif