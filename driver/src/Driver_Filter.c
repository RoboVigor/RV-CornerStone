#include "Driver_Filter.h"
#include "macro.h"

void Filter_Update(Filter_Type *filter, float value) {
    int i;
    // 初始化
    if (filter->count == 0) {
        filter->count     = 1;
        filter->diff      = 0;
        filter->offset    = 0;
        filter->value     = value;
        filter->lastValue = value;
        filter->result    = value;
        if (filter->windowSize > 0) {
            filter->movingAverageArray = malloc(4 * filter->windowSize);
            for (i = 0; i < filter->windowSize; i++) {
                *(filter->movingAverageArray + i) = value;
            }
        }
        return;
    }

    // 更新数据
    filter->lastValue = filter->value;
    filter->value     = value;
    filter->diff      = filter->value - filter->lastValue;
    filter->count += 1;
    *(filter->movingAverageArray + filter->count % filter->windowSize) = value;
}

void Filter_Update_Sample(Filter_Type *filter) {
    filter->average = (float) (filter->movingAverage * (filter->count - 1) + filter->value) / (float) filter->count;
    filter->max     = MAX(filter->max, filter->value);
    filter->min     = MIN(filter->min, filter->value);
}

void Filter_Update_Moving_Average(Filter_Type *filter) {
    int   i;
    float sum;
    for (i = 0; i < filter->windowSize; i++) {
        sum += *(filter->movingAverageArray + i);
    }
    filter->movingAverage = sum / (float) filter->windowSize;
}

float Filter_Apply_Limit_Breadth(Filter_Type *filter) {
    // 限幅
    if (ABS(filter->diff) < ABS(filter->thresholdLB)) {
        filter->offset -= filter->diff;
    }

    // 结果
    filter->result = filter->value + filter->offset;

    return filter->result;
}
