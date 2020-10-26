# 电控开发指南

CornerStone通用的代码风格在`.clang-format`文件里定义好了，按照[电控开发入门](page/best_practice)中所述的配置好VS Code，保存时就能自动格式化代码。超好用哟(*ﾟ∇ﾟ)

对变量名、函数名进行批量修改时请活用编辑器的全局替换功能。

一些规范中(没提到||需要重视||需要更改||需要讨论)的可以在[issues](https://github.com/RoboVigor/RV-CornerStone/issues)提出。

**Driver开发时务必遵守本规范，否则pull request不予通过**。User中的车种适配代码（除开common和template文件夹）可适当放宽要求。

### 版本号

v主版本号.次版本号.修订号
- 主版本号：一般在对代码结构进行了大量修改时迭代。主版本号迭代是不向下兼容的（所有代码必须重新适配）。在主版本号迭代时作者应提供详尽的代码结构变化说明以及适配指南（在pull request和[Wiki-适配指南](https://github.com/RoboVigor/RV-CornerStone/wiki/%E9%80%82%E9%85%8D%E6%8C%87%E5%8D%97)里更新）。
- 次版本号：一般在Driver新增功能时迭代。~~原则上属于向下兼容的版本（原有的User代码不需要进行修改）~~。实践证明对Driver进行修改时很难保证向下兼容。
- 修订版本：修正bug及更新User时迭代。向下兼容。

### 分支

robot-xxx 机器人分支。用于针对User的开发，特别强调，**在该分支下严禁修改Driver**。  
feature-xxx 功能分支。用于Driver或通用User代码的开发，在并入主分支前应上机测试。  
test-xxx 测试分支。临时测试用的代码，不并入主分支。
hotfix-xxx 热修复分支。用于及时修复develop分支上存在问题，应该在合并入主分支后直接删除。  

**任何分支并入主分支前必须提交Pull Request，由仓库管理员批准后方可并入。**

### 向下兼容

从CornerStone v1.0.0起，所有车种的代码将在User文件夹下统一发行，因此对于Driver的修改务必慎重，否则将导致其它车种无法通过编译/功能不正常。为了实现向下兼容，应在进行Driver开发时注意以下几点:

1. 不在Driver中使用任何`handle.h`中定义的全局变量（所以也不应在Driver中引入`handle.h`）。所有Driver中需要的变量全部通过函数传入或传出（详见编程风格部分）。
1. 不在Driver中定义任何中断、任务、全局变量等应该在User中定义的内容。
1. 文件内的局部变量必须使用static修饰词。
1. 应在[Wiki-适配指南](https://github.com/RoboVigor/RV-CornerStone/wiki/%E9%80%82%E9%85%8D%E6%8C%87%E5%8D%97)中提供Driver的使用方法。使用方法应以`user/template`为模板进行说明。此外，较为通用的功能应直接在template中写上使用范例并提供开关。

### 编程风格

C没有对象和类，但我们就是要实现抽象的对象和类。

Driver里的代码就是类，结构体就是数据结构（类的属性和特性）；`handle.c`和`handle.h`里的代码就是实例化并初始化对象；`tasks.c`里实现具体功能；`interrupt.c`定义中断（外部信号的处理）

所以Driver里的所有函数都应该包装成Package_Func(handle, arg1, arg2, [...])这样的形式，handle接受一个结构体（实例化对象），所以在Driver里的代码里不应该声明任何一个结构体变量。

只有使用车种多、算法较为固定而复杂的代码会整理成Driver，其余的业务逻辑应在`tasks.c`中编写。

### 命名方式

1. 变量命名采用**小驼峰型**，例：`roboVigor`
1. 类、结构、枚举、数组采用**大驼峰型**，例：`RoboVigor`
1. 前两条所述的类型另一种可选的声明方式是采用`<类型名>_<大驼峰型变量名>`，例：`TaskHandle_MoYu`
1. 类型应以`_t`或`_Type`结尾
1. 常量命名采用**全大写+单词间下划线**，例：`ROBO_VIGOR`
1. 函数命名采用**大驼峰型+单词间下划线**，优先以以下格式命名：
<工具/实现方式>\_<动作>\_<对象>\_<目标量>
例：`Can_Get_Motor_Speed`、`Usart_Get_Referee_Data`

以下是一些常用命名建议（范例中的<工具/实现方式>为Package）：
- `Package_Init`
- `Package_Update(_Something)`
- `Package_Get(_Something)`
- `Package_Calc(_Something)`
- `Package_Apply(_Something)`

其它命名请多参考现有的Driver代码，尽量做到命名的统一。

### 特有名词

对于一些特有名词如`D-Bus`, `USART`：
- 在命名中诸如驼峰型等命名规则优先度应凌驾于特有名词之上。~~然后我一时间并没有找到例子~~
- 在文件名里全大写。

### 头文件和源文件

- 头文件（.h）应该放在`/inc`文件夹下，源文件（.c）应该放在`/src`文件夹下。
- 结构体定义和函数原型应该写在头文件中；临时变量应该直接在源文件里声明，并使用`static`修饰词。

### 符号

代码及注释统一使用以下符号：
- 括号 `()` `[]` `{}` `<>`
- 引号 `""` `''`
- 逗号 `,`
- 句号 `.`
- 分号 `;`

### 参考

[document/华工代码规范.pdf](https://github.com/tccoin/RV-CornerStone/blob/develop/document/%E5%8D%8E%E5%B7%A5%E4%BB%A3%E7%A0%81%E8%A7%84%E8%8C%83.pdf)