# Point Cloud Grasp
<p align="left">
  <a href="## Project Overview">Paper</a> •
  <a href="https://github.com/JisuHann/Point-Cloud-Grasp">Github</a> 
</p>

<p align="left">
  <a href="## Project Overview">Project Overview</a> •
  <a href="## How to Use">How To Use</a> •
  <a href="## Contributors">Contributors</a> •
  <a href="## References">References</a> •
  <a href="## License">License</a> 
</p>

## Project Overview

본 연구는 딥러닝 알고리즘으로부터 손잡이를 인식하고, 손잡이 형태를 고려한 매니퓰레이터의 최적의 역기구학 해를 제시합니다.  
역기구학 해는 매니퓰레이터의 조작도와 손잡이 회전축과 파지점 사이의 거리를 고려한 목적 함수의 비교를 통해 선정됩니다.    
제안된 알고리즘을 Franka Panda로봇 팔에 적용하여 그 결과를 시뮬레이션을 통해서 성능을 평가합니다. 

### Algorithm Pseudo Code
<img src = "./algorithm.png" width="650">

### Demonstration  


## How to Use
본 연구는 Ubuntu 18.04 / ROS Melodic 환경에서 실행되었습니다.
환경 세팅을 위해 다음과 같은 Installation이 필요합니다.
- CoppeliaSim

## Contributors
이동규 [@Doroco](https://github.com/Doroco)  
한지수 [@JisuHann](https://github.com/JisuHann)  

## References
- "Multimodal templates for real-time detection of textureless objects in heavily cluttered scenes" [Paper](http://far.in.tum.de/pub/hinterstoisser2011linemod/hinterstoisser2011linemod.pdf)
-	"Yolov3: An incremental improvement" [Paper](https://pjreddie.com/media/files/papers/YOLOv3.pdf)
-	"Manipulability analysis" [Paper](https://ieeexplore.ieee.org/document/6651576)
-	"3D is here: Point Cloud Library (PCL)" [Paper](https://ieeexplore.ieee.org/document/5980567)


## License 
See the file [LICENSE](https://github.com/JisuHann/Point-Cloud-Grasp/blob/main/LICENSE) for copying permission. LICENSE를 참고하세요.
