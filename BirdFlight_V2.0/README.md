# BirdFlight飞控系统
<div align="center">
    <img src="http://server.xiluna.com:10080/Xiluna/Image/blob/master/BirdFlightOutdoor_V2.0/DRONE.jpg" width = "80%" /><br><br>
</div>

-----------------

| **代码编译** | **平衡实验** | **黑点实验** | **跟随小车** | **黑线追踪** | **光流遥控** | **地图构建** | 
|-----------------|---------------------|---------------------|---------------------|---------------------|---------------------|---------------------|          
|<img src="http://server.xiluna.com:10080/Xiluna/Image/raw/e20fb640d22ff9d65e0ff0cf11f1766524113b9f/BirdFlight_V2.0/passing.png">|<img src="http://server.xiluna.com:10080/Xiluna/Image/raw/e20fb640d22ff9d65e0ff0cf11f1766524113b9f/BirdFlight_V2.0/passing.png">| <img src="http://server.xiluna.com:10080/Xiluna/Image/raw/e20fb640d22ff9d65e0ff0cf11f1766524113b9f/BirdFlight_V2.0/failing.png">| <img src="http://server.xiluna.com:10080/Xiluna/Image/raw/e20fb640d22ff9d65e0ff0cf11f1766524113b9f/BirdFlight_V2.0/failing.png">| <img src="http://server.xiluna.com:10080/Xiluna/Image/raw/e20fb640d22ff9d65e0ff0cf11f1766524113b9f/BirdFlight_V2.0/failing.png">| <img src="http://server.xiluna.com:10080/Xiluna/Image/raw/e20fb640d22ff9d65e0ff0cf11f1766524113b9f/BirdFlight_V2.0/failing.png">| <img src="http://server.xiluna.com:10080/Xiluna/Image/raw/e20fb640d22ff9d65e0ff0cf11f1766524113b9f/BirdFlight_V2.0/failing.png">| 

**BirdFlight_V2.0**是一款高性能，多接口的四旋翼飞控。其主控芯片使用了ST公司的**STM32F405**。这颗高性能单片机为**BirdFlight_V2.0**提供了强有力的计算平台。**姿态部分：**九轴数据分别采自于应美盛的**MPU6500**和ST的**LSM303**两颗传感器，其中**MPU6500**主要提供三轴加速度数据和三轴陀螺仪数据，**LSM303**主要提供三轴磁力计数据。**高度部分：**高度数据来源于两个部分，室内主要由超声波**US-100**提供，室外由高精度气压计**MS5611**提供。**室内定位：**主要由BirdVision提供xy方向上的位置数据。**室外定位:**主要由GPS模块**Ublox**提供世界坐标系下的位置数据。

### 说明与原理简介

前往wiki查看更多信息 ：http://server.xiluna.com:10080/Xiluna_Bird/BirdFlight_V2.0/wikis/home

### 问题与讨论

前往issue进行提问 : 
http://server.xiluna.com:10080/Xiluna_Bird/BirdFlight_V2.0/issues









