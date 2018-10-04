#include "Driver_Fire.h"

/* Question
   以 Fire 来说，所需函数只是初始化，而且其实并不必要，但是需要一个头文件来声明其结构体
*/

void Fire_StateInit(FireState_Type *FireState) {
    FireState->State_Frict = 0;
    FireState->State_Stir  = 0;
}
