<h1 align="center">SLAM 기반 자율 이동 로봇(AMR) 순찰 및 보안 시스템🚒 </h1>

<h2 align="center">진돗봇🐕 </h2>

## 개요
자율 이동 로봇(AMR) 보안 시스템은 두 대의 AI 기반 TurtleBot4 로봇(TurtleBot4A, TurtleBot4B)을 사용하여 보안 구역 내에서 자율 순찰, 화재 탐지 및 진압, 직원 대피 유도, 침입자 탐색 및 추적을 제공하도록 설계.
시스템은 두 대의 AMR이 독립적으로 작동하되, 특정 시나리오에서 협력하며 데이터를 현장에서 처리.


## 제작 기간 & 참여 인원
-2025/05/23~2025/06/05  5명


## 사용한 기술 (기술 스택)    
<img src="https://img.shields.io/badge/python-blue?style=for-the-badge&logo=python&logoColor=white">   <img src="https://img.shields.io/badge/ROS2-black?style=for-the-badge&logo=ros&logoColor=#22314E">   <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white">   <img src="https://img.shields.io/badge/YOLO-111F68?style=for-the-badge&logo=yolo&logoColor=white">   <img src="https://img.shields.io/badge/socket.io-010101?style=for-the-badge&logo=socket.io&logoColor=white">    <img src="https://img.shields.io/badge/SQLite-003B57?style=for-the-badge&logo=sqlite&logoColor=white">   <img src="https://img.shields.io/badge/Flask-3BABC3?style=for-the-badge&logo=flask&logoColor=white">  


## High Level Architecture Diagram
<img width="517" height="403" alt="image" src="https://github.com/user-attachments/assets/759cdf71-ec8d-481c-89d0-11d668ce095e" />

이 시스템은 두 대의 AMR(TurtleBot4A, TurtleBot4B)로 구성되며, 데이터 처리, 네비게이션, 화재 탐지, 위협 탐지, 대피 유도 및 경고가 모두 AMR에서 로컬로 수행.
AMR은 모니터링, 알림 및 수동 제어를 위해 PC의 사용자 인터페이스와 로컬 네트워크(Wi-Fi)를 통해 직접 통신.


## Detail Diagram
<img width="589" height="631" alt="image" src="https://github.com/user-attachments/assets/dc6041c8-99e5-498e-8070-7d7f6ead9118" />


## 내가 기여한 부분
### 시나리오 설계, High Level Architecture Diagram 구상
-SLAM, navigate등을 사용 시에 로봇이 어떤 기준과 방향으로 map을 인식할지 결정

### Patrol fire 구현 후 코드 설계 및 통합
-Turtlebot4A,B가 정기 순찰시 YOLO로 employee들의 좌표를 인식해서 저장 후, exit의 좌표도 저장, 침입자 실시간 감지. 문제 없을 시에만 다음 waypoint로 이동

-Turtlebot4가 fire detect시 fire 좌표 저장 후 공유 및 fire_flag 발신 후 fire extinguish를 사용해 화재진압

-화재 감지를 못 한 Turtlebot4는 저장된 직원 위치를 바탕으로 대피 유도 후 exit 봉쇄(침입자 도주 차단)

-화재 진압을 마친 Turtlebot4가 잔류 employee확인 후 침입자(방화범)을 tracking하여 단계별 대응 

### terminal창의 log들을 website와 synchronize하여 SQLite3, socketio 기반의 Database처리
-Database처리를 하여 일정 line이 넘어가면 기록을 볼 수 없는 termianl창의 log들을 새로고침 없이 실시간으로 저장

-YOLO에서 detect한 class name과 class number을 인덱스화 하여 class name별로 정보 수집 가능


## 🌟핵심 기능 (코드로 보여주거나 코드 링크)
-Detection 부분에서 문제 발생 시 원인을 명확히 분석하고, QoS 최적화 및 프레임 처리 개선 등 실질적인 성능 향상을 이끌어냈다.

-다양한 종류의 YOLO 모델을 사용해 객체인식률을 높이기 위한 노력을 많이 했다.

-Website와 Terminal에 출력되는 log를 동기화시키기 위해 socketio를 사용해 새로고침 없이 실시간으로 송출시켰다.

-원활한 ROS2 communication을 위해 여러 방법과 명령어를 찾고 적용시켜 통신속도를 향상시켰다.

   
## 🎯트러블슈팅 경험  
-YOLO detect시에 Turtlebot4가 움직이며 받는 모든 시각 정보를 실시간 프레임으로 처리하다보니 CPU에 과부하가 생겨 객체인식이 지연되거나 프레임이 누락되는 문제가 발생
또한 여러 callback이 동시에 동작하며 좌표 계산 중 또 다른 계산이 발생되는 문제점을 파악


-한명의 employee를 여러번 감지하는 문제 발생


-synchronize시 멀티스레드환경에서 SQLite 접근 오류



## 해결방법
-이를 개선하기 위해 Qos를 최적화시키며 최신 이미지만 처리하도록 qos_profile_sensor_data를 실행하고 callback간 경쟁 없이 독립적으로 순차 실행 시키기 위해 MutuallyExclusiveCallbackGroup을 활용.
객체 인식 좌표가 이전 좌표와 다를 때만 publish하게 만들어 필요한 값의 publish만 받을 수 있도록 하고모든 이미지를 추론하지 않고 3프레임 중 1개만 처리하도록 함.


-해 employee를 발견시 publish되는 좌표들의 평균값을 저장하도록 함.
amr의 시야가 다른곳을 향하여 해당 좌표가 3초간 publish되지 않을 시 좌표들의 평균값을 저장.


-check_same_thread=False를 추가해 멀티스레드 안전성을 높임.
예외 처리도 강화해 데이터베이스저장 실패 시 로그에 오류 기록.
monitor_logs에서 SocketIO 추가로 실시간 로그 전송 로직을 통합.


9. 회고 / 느낀 점
