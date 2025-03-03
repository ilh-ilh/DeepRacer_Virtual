# 2022 re:Invent Championship - Counterclockwise
# Best record(2 Laps) : 33.5 sec
import math
optimal_waypoints = [
    [0.257920, 0.863320],
    [0.118930, 1.018910],
    [-0.018740, 1.172840],
    [-0.175920, 1.348670],
    [-0.347980, 1.541510],
    [-0.529950, 1.745810],
    [-0.716930, 1.955960],
    [-0.909870, 2.173180],
    [-1.107130, 2.395620],
    [-1.307250, 2.621670],
    [-1.508070, 2.848820],
    [-1.714070, 3.067740],
    [-1.930820, 3.266410],
    [-2.159770, 3.432680],
    [-2.398240, 3.556470],
    [-2.640360, 3.631100],
    [-2.878560, 3.653130],
    [-3.104330, 3.621440],
    [-3.307990, 3.536040],
    [-3.477010, 3.397880],
    [-3.608460, 3.221060],
    [-3.698780, 3.014090],
    [-3.745190, 2.785020],
    [-3.746060, 2.542190],
    [-3.701900, 2.294260],
    [-3.615780, 2.049500],
    [-3.493390, 1.814440],
    [-3.355340, 1.610480],
    [-3.239380, 1.401430],
    [-3.151210, 1.185370],
    [-3.096680, 0.960200],
    [-3.084070, 0.723460],
    [-3.104210, 0.477910],
    [-3.153940, 0.224620],
    [-3.228970, -0.035260],
    [-3.323980, -0.300490],
    [-3.437330, -0.580920],
    [-3.545440, -0.863450],
    [-3.645220, -1.149160],
    [-3.733310, -1.438940],
    [-3.806130, -1.733160],
    [-3.859840, -2.031560],
    [-3.881080, -2.332310],
    [-3.857930, -2.626300],
    [-3.783990, -2.898840],
    [-3.660940, -3.134210],
    [-3.497190, -3.320070],
    [-3.304240, -3.448610],
    [-3.094410, -3.515120],
    [-2.880480, -3.515800],
    [-2.677900, -3.446090],
    [-2.496250, -3.321390],
    [-2.341240, -3.150050],
    [-2.215990, -2.939720],
    [-2.120370, -2.698580],
    [-2.049810, -2.435740],
    [-1.995080, -2.160830],
    [-1.932630, -1.884890],
    [-1.849750, -1.625210],
    [-1.737700, -1.393020],
    [-1.593720, -1.198560],
    [-1.420190, -1.050900],
    [-1.223340, -0.958790],
    [-1.013430, -0.931210],
    [-0.807290, -0.977860],
    [-0.619650, -1.084310],
    [-0.459690, -1.241650],
    [-0.333770, -1.441670],
    [-0.245500, -1.676020],
    [-0.194920, -1.935960],
    [-0.177500, -2.212850],
    [-0.146801, -2.546089],
    [-0.102358, -2.831794],
    [-0.022996, -3.054009],
    [0.055723, -3.218849],
    [0.204560, -3.417298],
    [0.357944, -3.533358],
    [0.558962, -3.623724],
    [0.763254, -3.682093],
    [0.978490, -3.714925],
    [1.222911, -3.751406],
    [1.441795, -3.769646],
    [1.675272, -3.776943],
    [1.967118, -3.773294],
    [2.193298, -3.762350],
    [2.419479, -3.733166],
    [2.671195, -3.693037],
    [2.893728, -3.663852],
    [3.108964, -3.627372],
    [3.265831, -3.558058],
    [3.419050, -3.459560],
    [3.543085, -3.331878],
    [3.641583, -3.175011],
    [3.710230, -3.015610],
    [3.710840, -2.770680],
    [3.645080, -2.512570],
    [3.516170, -2.263970],
    [3.338980, -2.042190],
    [3.128460, -1.850000],
    [2.895180, -1.682540],
    [2.640900, -1.528980],
    [2.397940, -1.357610],
    [2.165320, -1.171110],
    [1.941870, -0.971990],
    [1.726260, -0.762680],
    [1.516980, -0.545550],
    [1.312380, -0.322950],
    [1.110680, -0.097120],
    [0.917540, 0.121200],
    [0.734410, 0.327790],
    [0.563190, 0.520560],
    [0.404470, 0.698930],
]

# TOTAL_NUM_STEPS = 285 
ESCAPING_STEP_HAIRPIN_LEFT_0 = 68 # 4.52 * 15 = 68 steps
ESCAPING_STEP_HAIRPIN_LEFT_1 = 124 # 8.26 * 15 = 124 steps
ESCAPING_STEP_HAIRPIN_LEFT_2 = 162 # 10.8 * 15 = 162 steps
ESCAPING_STEP_MULTIPLE_CURVES = 199 # 13.26 * 15 = 198.9 steps

class CustomReward:
    def __init__(self, optimal_waypoints):
        self.prev_speed = 0.0
        self.prev_segment_type = None

        # optimal waypoints
        self.optimal_waypoints = optimal_waypoints
        self.num_optimal_waypoints = len(optimal_waypoints)

        ######### 직진류 #########
            ### 직진구간 ###
        self.straight_segments = [
                ## 2차 헤어핀 직전 ##
            range(33, 41), # 6
                ## 2차 헤어핀 탈출 후 ##
            range(54, 56), # 9 
                ## 마지막 헤어핀 직전 구불한 경로 ## 
            range(79, 86), # 14
        ]
            ### 처음, 끝부분 긴 직진구간 ### 
        self.autobahn_segments = [
            range(0, 12), # 0
            range(98, 111) # 17
        ]
        ######### 직진류 #########


        ######### 좌회전류 #########
            ### 3차 헤어핀 탈출 후 좌회전 진입구간 ### 
        self.left_turn_first_half = [
            range(71, 76), # 12
            range(86, 92) # 15, must on left side of the track
        ]
            ### 좌회전 구간의 apex ###
        self.left_turn_apex=[
            range(75, 76), # point 75
            range(91, 92) # point 91
        ]
            ### 3차 헤어핀 탈출 후 좌회전 탈출구간 ###
        self.left_turn_second_half = [
            range(76, 77), # 13, must on left side of the track
            range(92, 98) # 16
        ]
            ### 1, 2차 헤어핀 진입구간 ###
        self.left_hairpin_first_half = [ 
            range(12, 19), # 2
            range(41, 50) # 7
        ]
            ### 1차 헤어핀 apex ###
        self.left_hairpin_apex = [
            range(19, 20) # point 19
        ]
            ### 1, 2차 헤어핀 탈출구간 ###
        self.left_hairpin_second_half = [
            range(19, 25), # 3, must on left side of the track
            range(50, 54), # 8, must on left side of the track
        ]
        ######### 좌회전류 #########


        ######### 우회전류 #########
            ### 1차 헤어핀 탈출 후 완만한 우회전 진입구간 ###
        self.right_turn_first_half = [
            range(25, 30) # 4
        ]
            ### 우회전 구간의 apex ###
        self.right_turn_apex = [
            range(30, 31), # point 30
            range(82, 83) # point 82. 실제 우회전 구간은 아니지만 그 점을 지나야 함
        ]
            ### 1차 헤어핀 탈출 후 완만한 우회전 탈출구간 ### 
        self.right_turn_second_half = [
            range(30, 33) # 5, must on right side of the track
        ]
            ### 3차 헤어핀 진입구간 ###
        self.right_hairpin_first_half = [
            range(56, 64) # 10
        ]
            ### 3차 헤어핀 apex ###
        self.right_hairpin_apex = [
            range(63, 64) # point 63
        ]
            ### 3차 헤어핀 탈출구간 ###
        self.right_hairpin_second_half = [
            range(64, 71), # 11, must on right side of the track
        ]
        ######### 좌측 차선에 있어야 하는 구간 #########
        self.must_on_left = [
            range(19, 25),
            range(50, 54),
            range(73, 77),
            range(86, 92),
        ]
        ######### 우측 차선에 있어야 하는 구간 #########
        self.must_on_right = [
            range(28, 32),
            range(59, 70),
            range(80, 84)
        ]

    def total_reward(self, params: dict) -> float:
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']

        # (1) 트랙 이탈, 충돌 등 패널티
        if params["is_offtrack"] or params["is_crashed"]:
            return 1e-6

        # (2) 현재 차량 위치에서 가장 가까운 optimal waypoint 찾기
        car_x, car_y = params["x"], params["y"]
        closest_idx = self._find_closest_waypoint_index(car_x, car_y, self.optimal_waypoints)
        prev_idx = (closest_idx - 1) % self.num_optimal_waypoints
        next_idx = (closest_idx + 1) % self.num_optimal_waypoints

        # (3) 현재 세그먼트 판별
        segment_type = self._get_segment_type(next_idx % 112)
        left_or_right = self._get_left_or_right(next_idx % 112)

        # (4) distance 보상
        distance_reward = self._distance_reward(car_x, car_y, prev_idx, next_idx, segment_type, params)

        # (5) heading 보상 (차량 전방 3번째 waypoint를 목표로 함)
        heading_reward = self._heading_reward(params, closest_idx, car_x, car_y, segment_type)

        # (6) 세그먼트별 추가 보상
        segment_reward = 0.0
        segment_reward += self._segment_specific_reward(params, segment_type, left_or_right)

        # (7) left or right 보상(계산에 곱으로 적용)
        left_or_right_scale_factor = 1.0
        left_or_right_scale_factor += self._left_or_right_reward(params, left_or_right)
        

        ########################## 최종 reward 합산 ##########################
        reward = 1e-6
        reward += distance_reward
        reward += heading_reward
        reward += segment_reward

        # scale factor 적용 전 reward 음수 방지
        reward = max(1e-6, reward)

        # left or right scale factor 적용
        reward *= left_or_right_scale_factor

        # 속도 느릴 시 패널티
        if speed < 1.1:
            reward *= 0.7
        if steps == ESCAPING_STEP_HAIRPIN_LEFT_0 and progress >= 26.5:
            reward += progress # = 26.5
        # 2차 헤어핀 탈출을 8.8초 이내에 하면 잭팟 보상
        if steps == ESCAPING_STEP_HAIRPIN_LEFT_1 and progress >= 49.2:
            reward += progress * 0.6 # 49.2 * 0.6 = 29.52
        # 3차 헤어핀 탈출을 11.5초 이내에 하면 잭팟 보상
        if steps == ESCAPING_STEP_HAIRPIN_LEFT_2 and progress >= 62.0:
            reward += progress * 0.4 # = 24.8
        if steps == ESCAPING_STEP_MULTIPLE_CURVES and progress >= 75.0:
            reward += progress * 0.5 # = 37.2
        if steps == 480 and progress >= 99.0:
            reward += progress * 0.5 # = 49.5
        
        # reward 음수 방지
        reward = max(1e-6, reward)
        ########################## 최종 reward 합산 ##########################

        # (8) 내부 변수 업데이트
        self.prev_speed = params["speed"]
        self.prev_segment_type = segment_type

        return float(reward)



    # (A) 차량 위치에 가장 가까운 waypoint 인덱스
    def _find_closest_waypoint_index(self, x, y, waypoints):
        closest_idx = 0
        min_dist = float("inf")
        for i, (wx, wy) in enumerate(waypoints):
            dist = (wx - x)**2 + (wy - y)**2
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx

    # (B) 세그먼트 타입 판별
    def _get_segment_type(self, waypoint_index: int) -> str:
        for seg_range in self.autobahn_segments:
            if waypoint_index in seg_range:
                return 'autobahn' # 긴 직진
        for seg_range in self.straight_segments:
            if waypoint_index in seg_range:
                return 'straight' # 짧은 직진
        for seg_range in self.left_turn_first_half:
            if waypoint_index in seg_range:
                return 'left_turn_first_half' # 진입
        for seg_range in self.left_turn_second_half:
            if waypoint_index in seg_range:
                return 'left_turn_second_half' # 탈출
        for seg_range in self.left_hairpin_first_half:
            if waypoint_index in seg_range:
                return 'left_hairpin_first_half' # 진입
        for seg_range in self.left_hairpin_second_half:
            if waypoint_index in seg_range:
                return 'left_hairpin_second_half' # 탈출
        for seg_range in self.right_turn_first_half:
            if waypoint_index in seg_range:
                return 'right_turn_first_half' # 진입
        for seg_range in self.right_turn_second_half:
            if waypoint_index in seg_range:
                return 'right_turn_second_half' # 탈출
        for seg_range in self.right_hairpin_first_half:
            if waypoint_index in seg_range:
                return 'right_hairpin_first_half' # 진입
        for seg_range in self.right_hairpin_second_half:
            if waypoint_index in seg_range:
                return 'right_hairpin_second_half' # 탈출
            
        for seg_range in self.left_turn_apex:
            if waypoint_index in seg_range:
                return 'left_turn_apex' # apex
        for seg_range in self.right_turn_apex:
            if waypoint_index in seg_range:
                return 'right_turn_apex' # apex
        for seg_range in self.left_hairpin_apex:
            if waypoint_index in seg_range:
                return 'left_hairpin_apex' # apex
        for seg_range in self.right_hairpin_apex:
            if waypoint_index in seg_range:
                return 'right_hairpin_apex' # apex
        

    # (C) 차선 왼쪽 / 오른쪽에 있어야만 하는 구간인지 체크
    def _get_left_or_right(self, waypoint_index: int) -> str:
        for seg_range in self.must_on_left:
            if waypoint_index in seg_range:
                return 'must_on_left'
        for seg_range in self.must_on_right:
            if waypoint_index in seg_range:
                return 'must_on_right'
        else:
            return 'none'

    # (D) distance reward 계산
    def _distance_reward(self, car_x, car_y, prev_idx, next_idx, segment_type: str, params: dict) -> float:
        track_width = params["track_width"]
        speed = params["speed"]
        all_wheels_on_track = params["all_wheels_on_track"]
        is_offtrack = params["is_offtrack"]
        steering = params["steering_angle"]
        x1, y1 = self.optimal_waypoints[prev_idx]
        x2, y2 = self.optimal_waypoints[next_idx]

        dist = self._perpendicular_distance(car_x, car_y, x1, y1, x2, y2)

        distance_reward = 0.01
        if dist < track_width * 0.5: # distance가 track_width의 절반 이내에 있으면
            distance_reward = 1.5 * (1.0 - dist / track_width) # 모든 segment에서 dist에 따른 선형적 보상

            # autobahn은 distance보다 speed가 더 중요
            if segment_type == 'autobahn': 
                distance_reward *= 0.9 

            # 곡선 구간은 추가 보상 부여
            if (segment_type == 'left_hairpin_first_half' or 
               segment_type == 'right_hairpin_first_half' or 
               segment_type == 'left_hairpin_second_half' or 
               segment_type == 'right_hairpin_second_half' or 
               segment_type == 'left_turn_first_half' or 
               segment_type == 'left_turn_second_half'):
                    distance_reward += 1.0 
                    if dist < track_width * 0.1:
                        distance_reward += 2.5
                        if dist < track_width * 0.05:
                            distance_reward += 2.0
        else:
            distance_reward = 1e-6

        ######## apex를 잘 공략했으면 보상, 아니면 패널티 ########
        if segment_type == 'left_turn_apex' or segment_type == 'right_turn_apex':
            if all_wheels_on_track is False and is_offtrack is False:
                distance_reward += 1.0
            else:
                distance_reward += 0.0
        elif segment_type == 'right_hairpin_apex':
            if all_wheels_on_track is False and is_offtrack is False:
                distance_reward += 1.0
                if steering < -25:
                    distance_reward += 1.0
                else:
                    distance_reward -= 0.5
            else:
                distance_reward += 0.0
        elif segment_type == 'left_hairpin_apex':
            if all_wheels_on_track is False and is_offtrack is False:
                distance_reward += 1.0
                if steering > 25:
                    distance_reward += 1.0
                else:
                    distance_reward -= 0.5
            else:
                distance_reward += 0.0
        else:
            distance_reward += 0.0

        return distance_reward


    def _perpendicular_distance(self, px, py, x1, y1, x2, y2):
        line_vec = (x2 - x1, y2 - y1)
        p_vec = (px - x1, py - y1)

        line_len_sq = line_vec[0]**2 + line_vec[1]**2
        if line_len_sq == 0:
            # 두 waypoint 동일 좌표 예외 처리
            return math.hypot(px - x1, py - y1)

        t = (p_vec[0]*line_vec[0] + p_vec[1]*line_vec[1]) / line_len_sq
        t = max(0.0, min(1.0, t))

        proj_x = x1 + t * line_vec[0]
        proj_y = y1 + t * line_vec[1]

        dist = math.hypot(px - proj_x, py - proj_y)
        return dist

    # (E) heading 보상 (차량 전방 6번째 waypoint 기준으로 계산)
    def _heading_reward(self, params: dict, closest_idx: int, car_x: float, car_y: float, segment_type) -> float:
        # heading reward 목적 : oscillation 방지
            # distance reward 비중이 크면 oscillation이 발생할 수 있다고 판단

        # 로직 : 차량 heading이 6번째 앞 waypoint heading을 바라볼 때 보상을 최대로 줌
            # target heading: 차량 전방 6번째 waypoint를 바라보는 heading
            # current heading: 차량 현재 heading
            # heading_diff = target heading - current heading (헤딩 에러)

        heading_reward = 0.0
        # 6번째 앞 waypoint 인덱스
        lookforward_number = 6
        target_idx = (closest_idx + lookforward_number) % self.num_optimal_waypoints
        target_x, target_y = self.optimal_waypoints[target_idx]

        dx = target_x - car_x
        dy = target_y - car_y
        target_heading_deg = math.degrees(math.atan2(dy, dx))

        current_heading_deg = params["heading"]
        heading_diff = abs(self._normalize_angle(target_heading_deg - current_heading_deg))


        # 먼저 0~60 구간으로 clamp
        heading_diff_clamped = max(0.0, min(60.0, heading_diff))

        # ratio = (최소값 / 최대값) = (1e-2 / 2.0) = 0.005
        ratio = 0.005

        # exponent = heading_diff_clamped / 60
        exponent = heading_diff_clamped / 60.0

        # 지수함수로 보상값 계산: 1.5 * (0.005 ^ exponent)
        heading_reward = min(2.0, 1.5 * (ratio ** exponent))

        # 만약 heading_diff 60도 초과면 최소값(1e-3)
        if heading_diff > 60.0:
            heading_reward = 1e-3
        # 급커브 구간에서만 보상 부여
        if segment_type == 'straight' or segment_type == 'autobahn' or segment_type == 'right':
            return 0.0

        return heading_reward

    def _normalize_angle(self, angle: float) -> float:
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle


    # (F) 세그먼트별 보상/패널티
    def _segment_specific_reward(self, params: dict, segment_type: str, left_or_right: str) -> float:
        reward = 0.0
        speed = params["speed"]
        steering = params["steering_angle"]

    
        if segment_type == 'autobahn' :
            if speed >= 3.0:
                reward += 1.0
                if speed >= 3.5:
                    reward += 3.8
            else:
                reward -= 1.0
            if steering == 0:
                reward += 1.0
            else:
                reward -= 0.5

        elif segment_type == 'straight':
            if speed > 2.0:
                reward += 2.5
            if abs(steering) > 20:
                reward -= 0.75

        elif segment_type == 'straight':
            if speed > 2.0:
                reward += 2.5
            elif speed < 1.4:
                reward -= 0.5

        elif segment_type == 'left_hairpin_first_half':
            ######## steering 범위 ########
            if steering > 20:
                reward += 1.5
            elif 0 <=steering <= 20:
                reward += 0.25
            else: # steering < 0
                reward -= 1.5
            # ######## speed 범위 ########

            # if speed > 2.0:
            #     reward -= 1.0
            # elif 1.5 <= speed <= 2.0:
            #     reward -= 0.5
            # elif 1.1 <= speed < 1.5:
            #     reward += 1.5
            # else:
            #     reward += -1.0


        elif segment_type == 'left_hairpin_second_half':
            ######## steering 범위 ########
            if steering >= 15:
                reward += 1.5
            elif 0 <= steering <= 15:
                reward + 0.25
            else:
                reward -= 1.5
            # ######## speed 범위 ########
      
            # if speed > 2.0:
            #     reward -= 1.5
            # elif 1.7 <= speed <= 2.0:
            #     reward -= 0.5
            # elif 1.2 <= speed < 1.7:
            #     reward += 1.5
            # else:
            #     reward -= 1.0
            
        elif segment_type == 'right_hairpin_first_half':
            ######## steering 범위 ########
            if steering < -25:
                reward += 1.5
            elif -25 <=steering <= 0:
                reward -= 0.75
            else: # steering < 0
                reward -= 1.5
            # ######## speed 범위 ########
            
            # if speed > 2.0:
            #     reward -= 1.5
            # elif 1.5 <= speed <= 2.0:
            #     reward -= 1.0
            # elif 1.0 <= speed < 1.5:
            #     reward += 1.5
            # else:
            #     reward += -1.0
        
        elif segment_type == 'right_hairpin_second_half':
            ######## steering 범위 ########
            if steering < -25:
                reward += 1.5
            elif -25 <= steering <= 0:
                reward -= 0.75
            else:
                reward -= 1.5
            # ######## speed 범위 ########
            # if speed > 2.2:
            #     reward -= 1.0
            # elif 1.8 <= speed <= 2.2:
            #     reward -= 0.5
            # elif 1.2 <= speed < 1.8:
            #     reward += 1.5
            # else:
            #     reward -= 1.0

        elif segment_type == 'left_turn_first_half':
            ######## steering 범위 ########
            if steering > 20:
                reward += 1.5
            elif 0 <=steering <= 20:
                reward += 0.25
            else: # steering < 0
                reward -= 1.0

                reward += 0.0
            # ######## speed 범위 ########
            # if speed > 2.0:
            #     reward -= 1.0
            # elif 1.7 <= speed <= 2.0:
            #     reward -= 0.5
            # elif 1.2 <= speed < 1.7:
            #     reward += 1.0
            # else:
            #     reward += -1.0

        elif segment_type == 'left_turn_second_half':
            ######## steering 범위 ########
            if steering >= 15:
                reward += 1.5
            elif 0 <= steering <= 15:
                reward += 0.25
            else:
                reward -= 1.5
            # ######## speed 범위 ########
            # if speed < 1.4:
            #     reward -= 1.0
            # else:
            #     reward += 1.0

        elif segment_type == 'right_turn_first_half':
            ######## steering 범위 ########
            if steering > 0:
                reward -= 1.0
            else:
                reward += 0.5
            # ######## speed 범위 ########
            if speed > 1.7:
                reward -= 1.0
            elif speed < 1.2:
                reward -= 1.0
            else:
                reward += 0.75

        elif segment_type == 'right_turn_second_half':
            ######## steering 범위 ########
            if steering > 0:
                reward -= 1.0
            else:
                reward += 0.5
            # ######## speed 범위 ########
            if speed < 1.3:
                reward -= 1.0
            else:
                reward += 0.75




        reward = max(-3.0, reward)
        return reward


    # (G) left or right 보상
    def _left_or_right_reward(self, params: dict, left_or_right: str) -> float:
        scale_factor = 0.0
        steering = params["steering_angle"]
        is_left_of_center = params["is_left_of_center"]
        is_right_of_center = not params["is_left_of_center"]

        ######### 차선 좌우 위치에 따른 보상 ######### 
        if left_or_right == 'must_on_left':
            if is_left_of_center:
                 scale_factor = 1.2
            else:
                scale_factor = 0.5
        elif left_or_right == 'must_on_right':
            if is_right_of_center:
                scale_factor = 1.2
            else:
                scale_factor = 0.5
        else:
            return 1.0
        return scale_factor     


my_custom_reward = CustomReward(optimal_waypoints)

def reward_function(params: dict) -> float:
    return my_custom_reward.total_reward(params)
