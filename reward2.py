# Smile Speedway - Counterclockwise
# Race type : Object avoidance
# Best record(2 Laps) : 19.65 sec
import math
optimal_waypoints = [
    [-4.010645, -0.014905],
    [-4.020178, -0.144567],
    [-4.028880, -0.250508],
    [-4.030250, -0.449748],
    [-4.003255, -0.753799],
    [-3.934678, -1.050263],
    [-3.817976, -1.331079],
    [-3.651177, -1.585391],
    [-3.439085, -1.801485],
    [-3.186004, -1.975529],
    [-2.890534, -2.107129],
    [-2.573619, -2.197948],
    [-2.252796, -2.260921],
    [-1.934517, -2.311606],
    [-1.626920, -2.364579],
    [-1.330916, -2.428511],
    [-1.027537, -2.511572],
    [-0.704047, -2.623040],
    [-0.372394, -2.744747],
    [-0.055335, -2.838795],
    [0.253094, -2.904445],
    [0.558836, -2.940030],
    [0.863353, -2.941592],
    [1.165059, -2.905660],
    [1.459540, -2.830822],
    [1.743633, -2.717518],
    [2.011612, -2.570247],
    [2.261104, -2.394654],
    [2.492383, -2.196449],
    [2.707406, -1.981095],
    [2.909120, -1.753358],
    [3.100287, -1.516705],
    [3.281918, -1.272641],
    [3.454399, -1.022021],
    [3.617921, -0.765468],
    [3.772551, -0.503463],
    [3.917993, -0.236246],
    [4.053269, 0.036254],
    [4.175527, 0.314795],
    [4.279893, 0.600510],
    [4.360608, 0.894036],
    [4.411494, 1.194884],
    [4.426301, 1.501015],
    [4.399120, 1.807873],
    [4.287960, 1.997007],
    [4.174031, 2.183437],
    [4.018673, 2.307723],
    [3.811528, 2.463082],
    [3.666527, 2.587368],
    [3.459383, 2.701297],
    [3.272953, 2.773798],
    [2.951880, 2.856656],
    [2.568663, 2.877370],
    [2.154375, 2.867013],
    [1.791872, 2.794512],
    [1.491513, 2.690940],
    [1.232583, 2.545939],
    [0.973653, 2.400938],
    [0.735437, 2.193794],
    [0.517935, 1.976293],
    [0.269362, 1.820935],
    [0.041504, 1.675934],
    [-0.175998, 1.510218],
    [-0.361829, 1.402605],
    [-0.627735, 1.247611],
    [-0.911672, 1.126715],
    [-1.207515, 1.039869],
    [-1.516557, 0.982953],
    [-1.846273, 0.952269],
    [-2.191778, 0.938511],
    [-2.527080, 0.917066],
    [-2.842375, 0.874018],
    [-3.117446, 0.795571],
    [-3.378292, 0.685487],
    [-3.614501, 0.548856],
    [-3.788184, 0.414541],
    [-3.946022, 0.215567],
    [-4.101381, -0.291936],
]



class CustomReward:
    def __init__(self, optimal_waypoints):

        self.prev_speed = 0.0
        self.prev_segment_type = None

        # 주어진 optimal waypoints
        self.optimal_waypoints = optimal_waypoints
        self.num_optimal_waypoints = len(optimal_waypoints)

        # 세그먼트 구간 정의 (0~111 인덱스 기준)
        # 2바퀴 도니까 % 153 로 동일 구간을 반복한다고 가정
        self.straight_segments = [
            range(0, 3),    
            range(9, 19),      
            range(26, 40),  
            range(67, 71),
        ]
        self.left_turn_segments = [
            range(3, 9),  
            range(19, 26),   
            range(40, 45),
            range(45, 50),  
            range(50, 57),   
            range(57, 62),   
            range(71, 77)
        ]
        self.right_turn_segments = [
            range(62, 67)   
        ]

    def total_reward(self, params: dict) -> float:

        speed = params["speed"]
        #-----------------------------------------------------------------------------------#
        # (1) 트랙 이탈, 충돌 등 기본 패널티 처리
        #-----------------------------------------------------------------------------------#
        if params["is_offtrack"] or params["is_crashed"]:
            return 1e-6
        # elif not params["all_wheels_on_track"]:
        #     return 1e-3
        
        #-----------------------------------------------------------------------------------#
        # (2) 차량의 현재 위치에서 최적 waypoints( optimal_waypoints ) 중
        #     가장 가까운 waypoint index를 찾아 prev, next index 설정
        #-----------------------------------------------------------------------------------#
        car_x, car_y = params["x"], params["y"]
        closest_idx = self._find_closest_waypoint_index(car_x, car_y, self.optimal_waypoints)
        prev_idx = (closest_idx - 1) % self.num_optimal_waypoints
        next_idx = (closest_idx + 1) % self.num_optimal_waypoints

        #-----------------------------------------------------------------------------------#
        # (3) 현재 세그먼트(직선 / 좌회전 / 우회전) 판별
        #     - next_idx를 112로 모듈로 연산
        #-----------------------------------------------------------------------------------#
        segment_type = self._get_segment_type(next_idx % 153)

        #-----------------------------------------------------------------------------------#
        # (4) distance 보상: prev_idx, next_idx를 이은 선분과 (car_x, car_y)의 수직 거리
        #-----------------------------------------------------------------------------------#
        distance_reward = self._distance_reward(car_x, car_y, prev_idx, next_idx, params)

        #-----------------------------------------------------------------------------------#``
        # (5) heading 보상: 해당 세그먼트의 끝 인덱스 방행 vs. 차량 heading
        #-----------------------------------------------------------------------------------#
        heading_reward = self._heading_reward(params, closest_idx, car_x, car_y)

        #-----------------------------------------------------------------------------------#
        # (6) 세그먼트별 추가 보상 (가속 보상, 스티어링 보상 등)
        #-----------------------------------------------------------------------------------#
        segment_reward = 0.0
        # (6-A) 코너 → 직선 전환 시 가속 보상
        # segment_reward += self._acceleration_reward(params, segment_type)
        # (6-B) 직선 구간: 속도 보상 / 좌우회전 구간: 스티어링 방향 보상/패널티
        segment_reward += self._segment_specific_reward(params, segment_type)

        #-----------------------------------------------------------------------------------#
        # (7) 최종 reward 합산
        #-----------------------------------------------------------------------------------#
        reward = 1.0  # 기본값(가중치 용도)
        reward += distance_reward
        reward += heading_reward
        reward += segment_reward
        if speed < 0.5:
            return 1e-6
        #-----------------------------------------------------------------------------------#
        # (8) 내부 변수 업데이트
        #-----------------------------------------------------------------------------------#
        self.prev_speed = params["speed"]
        self.prev_segment_type = segment_type
        reward = max(1e-6, reward)
        return float(reward)

    #----------------------------------------------------------------------------
    # (A) 차량 위치에 가장 가까운 optimal_waypoints 인덱스 찾기
    #----------------------------------------------------------------------------
    def _find_closest_waypoint_index(self, x, y, waypoints):
        """
        (x, y)와의 유클리드 거리(Euclidean distance)가 가장 작은 인덱스를 구함
        """
        closest_idx = 0
        min_dist = float("inf")
        for i, (wx, wy) in enumerate(waypoints):
            dist = (wx - x)**2 + (wy - y)**2
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx

    #----------------------------------------------------------------------------
    # (B) 어떤 segment(직선/좌회전/우회전)인지 판별
    #----------------------------------------------------------------------------
    def _get_segment_type(self, waypoint_index: int) -> str:
        """
        waypoint_index가 어느 세그먼트에 속하는지 판단하여
        'straight', 'left', 'right' 중 하나를 return
        """
        for seg_range in self.straight_segments:
            if waypoint_index in seg_range:
                return 'straight'
        for seg_range in self.left_turn_segments:
            if waypoint_index in seg_range:
                return 'left'
        for seg_range in self.right_turn_segments:
            if waypoint_index in seg_range:
                return 'right'
        # 혹시 어느 구간에도 속하지 않는 경우(예외 방지용)
        return 'straight'

    #----------------------------------------------------------------------------
    # (C) distance 보상: 선분(prev_idx->next_idx)와 (car_x, car_y) 사이의 수직 거리
    #----------------------------------------------------------------------------
    def _distance_reward(self, car_x, car_y, prev_idx, next_idx, params: dict) -> float:
        """
        최적 경로(optimal_waypoints)의 두 점(prev_idx, next_idx)을 이은 선분에
        현재 차량 위치가 얼마나 근접해 있는지에 따라 보상을 부여
        """
        distance_reward = 1.0
        x1, y1 = self.optimal_waypoints[prev_idx]
        x2, y2 = self.optimal_waypoints[next_idx]

        dist = self._perpendicular_distance(car_x, car_y, x1, y1, x2, y2)
        track_width = params['track_width']
        # dist=0 일 때 보상 최대(=3.0), dist 커질수록 급격히 줄어듦
        if dist < track_width * 0.5:
            distance_reward = distance_reward = 2.0 * (1.0 - dist / track_width)
            if dist < track_width * 0.25:
                distance_reward += 1.0
                if dist < track_width * 0.1:
                    distance_reward += 1.0
                    if dist < track_width * 0.05:
                            distance_reward += 1.0
        else:
            distance_reward = 2.0 * (1.0 - dist / track_width)
        

        return distance_reward

    def _perpendicular_distance(self, px, py, x1, y1, x2, y2):
        """
        점(px, py)에서 선분 (x1,y1)->(x2,y2)에 대한 수직 거리 계산
        """
        line_vec = (x2 - x1, y2 - y1)
        p_vec = (px - x1, py - y1)

        line_len_sq = line_vec[0]**2 + line_vec[1]**2
        if line_len_sq == 0:
            # 두 waypoint가 같은 위치라면
            return math.hypot(px - x1, py - y1)

        # 내적(dot)을 이용해 선분에의 투영 위치 계산
        t = (p_vec[0]*line_vec[0] + p_vec[1]*line_vec[1]) / line_len_sq
        t = max(0.0, min(1.0, t))

        proj_x = x1 + t * line_vec[0]
        proj_y = y1 + t * line_vec[1]

        dist = math.hypot(px - proj_x, py - proj_y)
        return dist

    # (D) heading 보상 (차량 전방 3번째 waypoint 기준으로 계산)
    def _heading_reward(self, params: dict, closest_idx: int, car_x: float, car_y: float) -> float:
        """
        차량이 현재 바라봐야 할 방향(= 전방 5번째 waypoint 방향)과
        실제 차량 heading 간의 차이를 기반으로 보상.
        """
        # 3번째 앞 waypoint 인덱스
        target_idx = (closest_idx + 5) % self.num_optimal_waypoints
        target_x, target_y = self.optimal_waypoints[target_idx]

        dx = target_x - car_x
        dy = target_y - car_y
        target_heading_deg = math.degrees(math.atan2(dy, dx))

        current_heading_deg = params["heading"]
        heading_diff = abs(self._normalize_angle(target_heading_deg - current_heading_deg))

        # heading_diff가 작을수록 보상↑
        if heading_diff > 30:
            heading_reward = 1e-2
        else:
            heading_reward = 2.0 - (heading_diff / 30.0)

        return heading_reward

    def _get_segment_end_index(self, idx: int, segment_list: list) -> int:
        """
        현재 idx가 속한 구간에서 '마지막 인덱스'를 리턴
        예: 직선 구간 0~12 => 마지막 인덱스=12
        """
        for seg_range in segment_list:
            if idx in seg_range:
                return max(seg_range)
        return idx

    def _normalize_angle(self, angle: float) -> float:
        """
        -180 ~ +180 범위를 벗어난 angle을 normalize
        """
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    #----------------------------------------------------------------------------
    # (E) 코너→직선 전환 가속 보상
    #----------------------------------------------------------------------------
    # def _acceleration_reward(self, params: dict, segment_type: str) -> float:
    #     reward = 0.0
    #     if (self.prev_segment_type in ['left', 'right']) and (segment_type == 'straight'):
    #         curr_speed = params["speed"]
    #         if (curr_speed > self.prev_speed) and (self.prev_speed > 0):
    #             accel_amount = curr_speed - self.prev_speed
    #             # 예: 가속량에 비례한 보상
    #             reward += 5 * accel_amount
    #     return reward

    #----------------------------------------------------------------------------
    # (F) 세그먼트별 보상/패널티
    #----------------------------------------------------------------------------
    def _segment_specific_reward(self, params: dict, segment_type: str) -> float:
        reward = 0.0
        speed = params["speed"]
        steering_angle = params["steering_angle"]  # 문제에서 +가 좌회전, -가 우회전이라 가정

        if segment_type == 'straight':
            # 속도가 4에 가까울수록 높게: (speed/4)^2
            # reward += 4 * (speed / 4.0)
            if speed > 2.5:
                reward += 1.5
                if speed > 3.5:
                    reward += 2.0


        # elif segment_type == 'left':
        #     # 좌회전 구간: steering_angle > 0 => +, steering_angle < 0 => 패널티
        #     if steering_angle > 0:
        #         reward += 2.0
        #     else:
        #         reward -= 1.0
        #     if speed < 1.2:
        #         reward += -2.0

        # else:  # 'right'
        #     # 우회전 구간: steering_angle < 0 => +, steering_angle > 0 => 패널티
        #     if steering_angle < 0:
        #         reward += 2.0
        #     else:
        #         reward -= 1.0
        #     if speed < 1.2:
        #         reward += -2.0

        return reward


my_custom_reward = CustomReward(optimal_waypoints)

def reward_function(params: dict) -> float:
    return my_custom_reward.total_reward(params)
