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

```
## 액추에이터에게 1.57 라디안 이동 명령을 한다.

rostopic pub /two_link/joint1_position_controller/command std_msgs/Float64 "data: 1.57" 
```



