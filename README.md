# ball_tracking

# kalman filter를 이용하여 공의 위치를 예측하는 코드

-----------------------------------------

## Description
kalman filter를 이용해서 공의 위치를 filtering 및 예측하는 코드이다.

공의 위치 데이터가 끊기거나 공이 사라지는 경우를 예측하기 위한 코드이다.

모든 단위는 m,sec 로 통일되어 있다.

### 1. topic

|구분|이름|자료형|설명|
|-----|---|---|---|
|publisher|/kf_ball_pos|geometry_msgs::Pos2D|filtering ball position|
|publisher|/kf_ball_vel|geometry_msgs::Pos2D|filtering ball velocity|
|publisher|/predict_ball_pos|geometry_msgs::Pos2D|ball position after 1 sec|
|Subscriber|/ball_position|geometry_msgs::Pos2D|raw ball position|
|Subscriber|/ball_detect|std_msgs::Bool|ball is dectected|

### 2. 작동 영상
<p align="center">
<img src=https://user-images.githubusercontent.com/48857469/187656000-aa3a6cc1-d3d7-42b1-94b5-0011c5a4d543.gif>
</p>
