# DeepRacer_Virtual


## 2022 re:Invent Championship - CCW(Time Trial)
### Action Space
- ±30° : 1.2 m/s, 1.6 m/s
- ±20° : 1.6 m/s, 1.9 m/s
- ±10° : 3 m/s
- 0° : 3 m/s, 4 m/s

### Reinforcement learning algorithm
- PPO

### Hyperparamter
#### First 4H : 
- Gradient descent batch size : 64
- Entropy : 0.01
- Discount factor : 0.98
- Loss type : Mean squared error
- Learning rate : 0.0003
- Number of experience episodes between each policy-updating iteration : 20
- Number of epochs : 10
- [Reward graph img]
  

#### Second 5H : 
- Loss type : Huber
- Learning rate : 0.0001
- [Reward graph img]

---

## Smile Speedway - CCW(Object Avoidance)
### Object avoidance rules
- Collision penalty : 5 seconds
- Number of objects : 3
  - Object positions : fixed, 25%(out), 50%(in), 75%(out)
- Obstacle attempts : 3



### Action Space
- ±30° : 1.7 m/s, 2.5 m/s
- ±15° : 2.5 m/s, 3.5 m/s
- 0° : 4 m/s

### Reinforcement learning algorithm
- PPO

### Hyperparamter
#### 4H : 
- Gradient descent batch size : 64
- Entropy : 0.01
- Discount factor : 0.98
- Loss type : Mean squared error
- Learning rate : 0.0003
- Number of experience episodes between each policy-updating iteration : 20
- Number of epochs : 10
- [Reward graph img]
  


