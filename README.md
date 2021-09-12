# two_link
자연스러운 모션 스터디용 2링크 1조인트 가제보 패키지


## 실행

### 가제보 실행하기

```
roslaunch two_link gazebo.launch 
```

### Trajectory Node
```
rosrun two_link trj_node
```

### Published Topics

- /two_link/joint1_position_controller/command : 링크 조인트에 위치한 액추에이터의 각도를 명령할 수 있는 topic
    - 2번 링크가 12시를 향할 때가 0.00이며, y축을 기준으로 CCW로 회전한다.


- /two_link/joint1_position_controller/state_value : 모터의 포지션
- /two_link/joint1_position_controller/state_value_dot : 모터의 속도


- /two_link/joint_states/position[0] : 모터 포지션
- /two_link/joint_states/velocity[0] : 모터 속도
- /two_link/joint_states/effort[0]   : 모터 토크


```
## 액추에이터에게 1.57 라디안 이동 명령을 한다.

rostopic pub /two_link/joint1_position_controller/command std_msgs/Float64 "data: 1.57" 
```

### 트레젝토리 생성법 변경하기 

- 43번째줄 주석 해제 빌드 - 3차 방정식(Cubic Poly) 
- 38번째줄 주석 해제 빌드 - 사다리꼴(Trapezoidal)

```
// Cubic
std::vector<double> q = PointToPoint(t, q_i, q_f, dq_i, dq_f, t_i, t_f); // 치역

// Trapezoidal
// std::vector<double> q = trapezoidalVelocityProfile(q_i, q_f, ddq_c, t_f, t);
```



