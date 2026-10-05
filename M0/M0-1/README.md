# M0-1
## M0-1-1 物理机安装Ubuntu
- 主要学习方法：跟随csdn上文章指引一步步进行，遇到问题时使用AI辅助解决
- 参考文章链接：  
1. https://blog.csdn.net/babailicsdn/article/details/126585640?fromshare=blogdetail&sharetype=blogdetail&sharerId=126585640&sharerefer=PC&sharesource=qq_59512969&sharefrom=from_link
2. https://blog.csdn.net/Hclam/article/details/148404947?fromshare=blogdetail&sharetype=blogdetail&sharerId=148404947&sharerefer=PC&sharesource=qq_59512969&sharefrom=from_link
### 问题一：  
- 问题：secureboot开启使得Ubuntu引导无法出现
- 解决：寻求csdn相关文章与AI帮助后关闭secureboot 
- 结果：顺利继续Ubuntu安装
- 使用AI：ChatGPT
### 问题二：  
- 问题：安装完成后显示boot仅剩0字节->进入Live Ubuntu排查，GParted误读分区以为/boot有110G从而误认为没有问题->重启后仍弹窗，使用df -h /boot确认实际确实已满（initrd.img-6.8.0-40高达126MB）>参考文章发布于2022年当时/boot所需体积较小
- 解决：先修改配置让系统只打包必要驱动，不再生成大文件->删掉旧的大文件->重新生成小的initrd
- 结果：占用率降到21%
- 使用AI：Grok，DeepSeek
### 问题二后续问题：  
- 问题：第二天重启后直接报错（Kernel panic - not syncing: VFS: Unable to mount root fs）->新生成的 initrd 体积虽小，但缺少关键硬盘驱动（NVMe）-
- 解决：与AI讨论后，放弃在原系统上修补，直接重装 Ubuntu,并把/boot增加到1G
- 结果：重装后顺利启动且没有报错
- 使用AI：DeepSeek
## M0-1-2 安装ROS2 Humble
- 主要学习方法：跟随csdn上文章指引一步步进行
- 参考文章链接：  
https://blog.csdn.net/weixin_55944949/article/details/140373710?ops_request_misc=elastic_search_misc&request_id=3e24baf99fd01419e370d5d8ac70855d&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-2-140373710-null-null.142^v102^pc_search_result_base4&utm_term=ROS2%20Humble安装&spm=1018.2226.3001.4187
## M0-1-3 安装Python环境(conda)
- 主要学习方法：跟随csdn上文章指引进行，并根据ChatGPT建议使用Miniconda同时跟随其指引安装
- 参考文章链接：  
https://blog.csdn.net/ramsey17/article/details/137643804?ops_request_misc=&request_id=&biz_id=102&utm_term=ubuntu中安装Python虚拟环境&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-137643804.142^v102^pc_search_result_base4&spm=1018.2226.3001.4187
## M0-1-4 安装C/CPP+VS Code及必要插件
- 主要学习方法：根据DeepSeek建议使用build-essent安装gcc等，而后根据csdn上文章安装VS Code及必要插件
- 参考文章链接：  
1. https://blog.csdn.net/Forever_change/article/details/134006968?ops_request_misc=&request_id=&biz_id=102&utm_term=ubuntu中安装C/CPP%20vscode&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-134006968.142^v102^pc_search_result_base4&spm=1018.2226.3001.4187
2. https://blog.csdn.net/u014361280/article/details/127986092
## M0-1-5 安装Git及创建新仓库
- 主要学习方法：跟随csdn上文章及推荐的学习视频一步步进行
- 参考文章链接：  
https://blog.csdn.net/blackcat0_0/article/details/147378341?ops_request_misc=elastic_search_misc&request_id=03cfe7fd300c1311e6662915c0a1f634&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-2-147378341-null-null.142^v102^pc_search_result_base4&utm_term=ubuntu中安装git&spm=1018.2226.3001.4187
## M0-1-6 远程登录学习
- 主要学习方法：跟随DeepSeek指引在同一局域网Wi-Fi下使用拯救者运行Ubuntu操控MacBook
- 实操过程：配置SSH免密登录->实现文件传输->实现VS Code远程开发->反向SSH互连（Mac连接拯救者）
### 问题一：    
- 问题：多SSH密钥冲突导致GitHub免密失效
- 解决：利用'ls -la ~/.ssh/'查看现有密钥，而后执行'nano ~/.ssh/config'进行配置，强制GitHub连接使用旧密钥，恢复认证
- 结果：GitHub得以重新成功连接
- 使用AI：DeepSeek
### 问题二：  
- 问题：VS Code Remote-SSH卡在"Opening Remote..."无法进入
- 解决：修改remote.SSh.localServerDownload选项修改为always，并使用Remote-SSH:Kill VS Code Server on Host...清理卡死状态，最后在Mac终端执行'rm -rf ~/.vscode-server'删除残留
- 结果：重启VS Code重新连接，成功进入远程环境
- 使用AI：DeepSeek
