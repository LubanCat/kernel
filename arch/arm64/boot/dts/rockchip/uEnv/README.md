### LubanCat uEnv文件说明

在U-boot启动时，通过SDRADC检测硬件ID引脚，并根据硬件ID值加载相应的uEnv.txt到U-boot环境变量中，

以此来实现不同板卡通用一个固件。

#### 支持板卡：

- LubanCat0 系列 基于RK3566
- LubanCat1 系列 基于RK3566
- LubanCat2 系列 基于RK3568
- LubanCat4 系列 基于RK3588s
- LubanCat5 系列 基于RK3588

#### uEnv文件命名规范

- 使用uEnv开头
- 使用.txt结尾
- 中间为板卡名称


#### 注意事项

- **切勿在Uboot终端中使用saveenv命令，这会覆盖uEnv.txt文件，导致系统无法启动，需要重新烧录固件。**
