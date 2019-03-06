#include "Driver_Filter.h"
#include "macro.h"

void Filter_Update(Filter_Type *filter, float value) {
    // 初始化
    if (filter->count == 0) {
        filter->count     = 1;
        filter->diff      = 0;
        filter->offset    = 0;
        filter->value     = value;
        filter->lastValue = value;
        filter->result    = value;
        return;
    }

    // 更新数据
    filter->lastValue = filter->value;
    filter->value     = value;
    filter->diff      = filter->value - filter->lastValue;
    filter->count += 1;
}

void Filter_Update_Sample(Filter_Type *filter) {
    filter->movingAverage = (float) (filter->movingAverage * (filter->count - 1) + filter->value) / (float) filter->count;
    filter->max           = MAX(filter->max, filter->value);
    filter->min           = MIN(filter->min, filter->value);
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
