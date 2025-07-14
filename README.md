<h1 align="center">SLAM 기반 자율 이동 로봇(AMR) 순찰 및 보안 시스템🚒 </h1>

<h2 align="center">진돗봇🐕 </h2>

## 시나리오
자율 이동 로봇(AMR) 보안 시스템은 두 대의 AI 기반 TurtleBot4 로봇(TurtleBot4A, TurtleBot4B)을 사용하여 보안 구역 내에서 자율 순찰, 화재 탐지 및 진압, 직원 대피 유도, 침입자 탐색 및 추적을 제공하도록 설계되었습니다. 시스템은 두 대의 AMR이 독립적으로 작동하되, 특정 시나리오에서 협력하며 데이터를 현장에서 처리합니다.

### 제작 기간 & 참여 인원
-2025/05/23~2025/06/05  5명


### 사용한 기술 (기술 스택)    
<img src="https://img.shields.io/badge/python-blue?style=for-the-badge&logo=python&logoColor=white">   <img src="https://img.shields.io/badge/ROS2-black?style=for-the-badge&logo=ros&logoColor=#22314E">   <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white">   <img src="https://img.shields.io/badge/YOLO-111F68?style=for-the-badge&logo=yolo&logoColor=white">   <img src="https://img.shields.io/badge/socket.io-010101?style=for-the-badge&logo=socket.io&logoColor=white">    <img src="https://img.shields.io/badge/SQLite-003B57?style=for-the-badge&logo=sqlite&logoColor=white">   <img src="https://img.shields.io/badge/Flask-3BABC3?style=for-the-badge&logo=flask&logoColor=white">  


### High Level Architecture Diagram
<img width="517" height="403" alt="image" src="https://github.com/user-attachments/assets/759cdf71-ec8d-481c-89d0-11d668ce095e" />

### Detail Diagram
<img width="589" height="631" alt="image" src="https://github.com/user-attachments/assets/dc6041c8-99e5-498e-8070-7d7f6ead9118" />

### 내가 기여한 부분
- 시나리오 설계, High Level Architecture Diagram 구상
- patrol node 구현 후 코드 설계 및 통합
- log창과 terminal창을 synchronize하여 database처리


### 핵심 기능 (코드로 보여주거나 코드 링크)
   -Detection 부분에서 문제 발생 시 원인을 명확히 분석하고,
    QoS 최적화 및 프레임 처리 개선 등 실질적인 성능 향상을
    이끌어냈다.
   -다양한 종류의 YOLO 모델을 사용해 객체인식률을 높이기
    위한 노력을 많이 했다.
   -Website와 Terminal에 출력되는 log를 동기화시키기 위해
    socketio를 사용해 새로고침 없이 실시간으로 송출시켰다.
   -원활한 ROS2 communication을 위해 여러 방법과
    명령어를 찾고 적용시켜 통신속도를 향상시켰다.
8. 트러블슈팅 경험 / 자랑하고 싶은 코드 
9. 회고 / 느낀 점
