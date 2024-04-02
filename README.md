### 代码开发  
pkgs --update     # 更新软件包
menuconfig			  # 配置工程  
scons --target=mdk5	# 生成MDK工程  
scons -j4				  # 编译  
scons –verbose		# 显示编译参数  
scons -c				  # 清除编译  

### 命令行  


### 解crash
.\tools\addr2line\win64\addr2line.exe -e .\bsp\stm32\gss-stm32f429-atk-apollo\build\keil\Obj\rt-thread.axf -afpiC 0800798e 08007968 08004fc0 080153e
