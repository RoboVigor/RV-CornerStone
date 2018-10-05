#include "Driver_Filter.h"
#include "macro.h"

void Filter_Update(Filter_Type *filter, float value) {
    // 初始化
    if (!filter->isInit) {
        filter->isInit    = 1;
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
}

float Filter_Limit_Breadth(Filter_Type *filter) {
    // 限幅
    if (ABS(filter->diff) < ABS(filter->thresholdLB)) {
        filter->offset -= filter->diff;
    }

    // 结果
    filter->result = filter->value + filter->offset;

    return filter->result;
}
