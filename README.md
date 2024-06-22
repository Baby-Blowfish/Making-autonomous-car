# Making-autonomous-car (2023.05 ~ 2023.08)

## 1. 개발 배경 및 목적 
    - 미래형 이동수단 중장기 교육 프로그램 및 미래형 이동수단 자작 경진대회 
    - 1/5 스케일 자율주행 전기 자동차 자작 설계 및 제작, 미래 자동차 관련 대학 간 공유 협력



## 2. 프로젝트 개요 

    - 하네스 차량 하드웨어 재구성 및 임베디드 시스템 환경 설정
    - Ubuntu 환경에서 ROS기반 GPS, 3D LiDAR, IMU 센서 제어
    - Web Cam에 가져온 이미지 OpenCV를 활용하여 라인 인식
    - GPS localization과 LiDAR Mapping으로 High Automation 구현
    - 실제 도로 환경을 구성한 경기장에서 교통신호에 따라 목적지까지 자율주행 테스트


## 3. 기술 스택 
![화면 캡처 2024-06-21 184413](https://github.com/Baby-Blowfish/Making-autonomous-car/assets/168509536/f8bd79f5-efbe-4925-a634-59b4a0cf7620)



## 4. 프로젝트 내 역할 : 펌웨어 개발/ 임베디드 개발

    - ATmega2560으로 조향 Servo 모터, 전륜과 후륜의 Encoder DC 모터 PID control
    - ATmega2560와 PC의 Ubuntu 간의 Serial 통신 제어 알고리즘 개발
    - ROS 기반 GPS localization과 IMU 센서로 차량 현재 위치 파악 및 전역 경로 계획 알고리즘 개발
    - 초음파 센서로 장애물 감지 및 지역 경로 계획 알고리즘 개발
    - Web CAM으로 차선, 보행자 신호, 교통 신호를 인식하여 행동 계획 알고리즘 개발



## 5. 성과 
    - 특별상 / 미래형 이동수단 자작 경진대회 / 영남대학교

## 6. 활동
![KakaoTalk_20240622_175604361](https://github.com/Baby-Blowfish/Making-autonomous-car/assets/168509536/269e3d2e-0f17-4d6b-b935-77334c6c38d4)
![KakaoTalk_20240622_175538083](https://github.com/Baby-Blowfish/Making-autonomous-car/assets/168509536/467caed9-154d-4f53-899a-5b6847cdbbf1)
![KakaoTalk_20240622_175510461](https://github.com/Baby-Blowfish/Making-autonomous-car/assets/168509536/c62d065e-52cc-4ff7-bc1b-5ad8eb8e7610)
![그림3](https://github.com/Baby-Blowfish/Making-autonomous-car/assets/168509536/3b987bb7-e6c8-4fe7-9918-f6405b6dd716)
![그림2](https://github.com/Baby-Blowfish/Making-autonomous-car/assets/168509536/90983bf0-046c-4142-a830-82eeb5954244)






