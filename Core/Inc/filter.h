#ifndef __FILTER_H
#define __FILTER_H

typedef struct
{
    int debug;
} LPassFilter;

typedef struct
{
    int debug;
} HPassFilter;

void LPassFilterInit(LPassFilter *);
void HPassFilterInit(HPassFilter *);

#endif
