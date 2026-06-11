# AVATAR 로봇 설정 정보

`avatar_read_write_node.cpp` 의 TODO 항목을 실제 하드웨어에 맞게 수정해주세요.

---

## 1. 모터 속도 단위 (`VEL_RPM_PER_UNIT`)

**파일:** `src/avatar_read_write_node.cpp` → `#define VEL_RPM_PER_UNIT`

현재 값: `1.0f` (1 unit = 1 RPM 가정)

| 모터 시리즈 | 단위 |
|---|---|
| Dynamixel Pro H-series (H54, H42) | 1 RPM/unit |
| Dynamixel Pro+ / P-series | 0.01 RPM/unit |
| Dynamixel MX28 (Poppy) | 0.229 RPM/unit |

**AVATAR 모터 모델명:**
```
BIG_MOTOR (1~5, 9~13, 20~21 — 200W/100W):  _________________
MIDDLE_MOTOR (6~8, 14~16, 18 — 20W):        _________________
SMALL_MOTOR (17, 19 — 10W):                 _________________
```

→ 확인 후 `VEL_RPM_PER_UNIT` 수정 및 `VELOCITY_LIMIT` / `MAX_PROFILE_VEL` 재조정

각 관절 인덱스 당 해당하는 모터의 이름(시리즈) 는 아래와 같아.
1. H54-200-S500-R:
2. H54-200-S500-R:​
3. H54-200-S500-R:​
4. H54-200-S500-R:​
5. H54-100-S500-R:​
6. H42-20-S300-R:​
7. H42-20-S300-R:​
8. H42-20-S300-R:​

9. PH54-200-S500-R:​
10. PH54-200-S500-R:​
11. PH54-200-S500-R:​
12. H54-100-S500-R:​
13. H54-100-S500-R:​
14. H42-20-S300-R:​
15. H42-20-S300-R:​
16. H42-20-S300-R:​

17. PM42-010-S260-R:​
18. H42-20-S300-R:​
19. PM42-010-S260-R:​
​
20. H54-200-S500-R:​
21. H54-100-S500-R:​

그리고 모터 종류 당 velocity limit의 단위 , velocity limit의 최댓값, Goal position의 180도 위치값이야.
H54-200-S500-R: ​0.01 RPM, 2900, 501,923​
PH54-200-S500-R:​ 0.01 RPM, 2900, 501,923​
H54-100-S500-R:​ 0.01 RPM, 2920, 501,923​
H42-20-S300-R:​ 0.01 RPM, 2920, 303,750​
PM42-010-S260-R:​ 0.01 RPM, 2600, 263,187

---

## 2. 최대 속도 (`VELOCITY_LIMIT`, `MAX_PROFILE_VEL`)

현재 값: `260` (기존 `torque_read_degree_write.cpp` 의 `GOAL_VEL=260` 에서 가져옴)

실제 모터의 안전 최대 속도를 RPM 단위로 확인 후 unit으로 변환:

최대 속도는 터미널에서 얘기했던대로 10000 정도로 잡으면 될 것 같아. 주석으로 600도/s에서 10,000 unit으로 환산하는 과정을 남겨줘.
그리고 MAX_PROFILE_VEL는 VELOCITY_LIMIT과 동일하고, 프레임 당 현재 각도에서 목표 각도까지의 각도 차이를 프레임 시간으로 나눠서 목표 속도 만큼 profile velocity를 조절해서 움직일 수 있도록 해줘.
```
vel_unit = RPM / VEL_RPM_PER_UNIT
```

**적용할 값:**
```
VELOCITY_LIMIT   = _______   (EEPROM 상한)
MAX_PROFILE_VEL  = _______   (소프트웨어 상한, VELOCITY_LIMIT 이하)
WARMUP_START_VEL = _______   (워밍업 시작 속도, 현재 10)
```

---

## 3. 모터별 각도 제한 (`CONST_DEG`)

아래 표를 실제 로봇 가동 범위에 맞게 수정해주세요.  
현재 값은 `torque_read_degree_write.cpp` 에서 가져온 것입니다.

| 모터 ID | 관절 | 링크 | 현재 min | 현재 max | 수정값 min | 수정값 max |
|---------|------|------|----------|----------|-----------|-----------|
| 1  | joint1  | R_Shoulder1 (Z) | -30 | 60 | | |
| 2  | joint2  | R_Shoulder2 (-Y) | -45 | 160 | | |
| 3  | joint3  | R_Shoulder3 (-X) | 0 | 160 | | |
| 4  | joint4  | R_Upperarm (-Z) | -90 | 90 | | |
| 5  | joint5  | R_Forearm (-Y) | 0 | 120 | | |
| 6  | joint6  | R_Wrist1 (-Z) | -90 | 90 | | |
| 7  | joint7  | R_Wrist2 (-Y) | -60 | 60 | | |
| 8  | joint8  | R_Wrist3 (-X) | -90 | 60 | | |
| 9  | joint9  | L_Shoulder1 (Z) | -60 | 30 | | |
| 10 | joint10 | L_Shoulder2 (+Y) | -160 | 45 | | |
| 11 | joint11 | L_Shoulder3 (-X) | -160 | 0 | | |
| 12 | joint12 | L_Upperarm (-Z) | -90 | 90 | | |
| 13 | joint13 | L_Forearm (+Y) | -120 | 0 | | |
| 14 | joint14 | L_Wrist1 (-Z) | -90 | 90 | | |
| 15 | joint15 | L_Wrist2 (+Y) | -60 | 60 | | |
| 16 | joint16 | L_Wrist3 (+X) | -90 | 60 | | |
| 17 | joint17 | Neck1 (Z) | -90 | 90 | | |
| 18 | joint18 | Neck2 (-Y) | -90 | 90 | | |
| 19 | joint19 | Head (X) | -90 | 90 | | |
| 20 | joint20 | Waist_Top (Y) | -30 | 30 | | |
| 21 | joint21 | Waist_Bottom (Z) | -30 | 30 | | |

위 값이 맞아.
---

## 4. CSV 퍼블리셔 설정

`avatar_read_write_node` 는 `/joint_states` 토픽에서 `position[0]~position[20]` 을
모터 1~21 의 목표 각도(radian)로 읽습니다.

CSV 파일 형식: 각 행이 21개의 각도값(degree 또는 radian) 을 콤마로 구분
- degree 인 경우: CSV 퍼블리셔에서 `* PI / 180` 변환 후 발행 필요
- radian 인 경우: 그대로 발행

**현재 사용할 CSV 퍼블리셔:**
```
(예: poppy_csv_joint_state_publisher 수정 버전 또는 새로 작성)
avatar_csv_joint_state_publisher 이름으로 수정해서 새로 작성해줘. 내용은 동일하되 로봇에 해당하는 정보만 수정하면 될 것 같아.
```

**CSV 데이터 폴더:**
```
(예: data/csv/avatar/)
data/csv/avatar/ 폴더에 csv 형식으로 몇개의 데이터 넣어놨어.
```

**CSV 컬럼 순서 (21개):**
```
joint1(M1), joint2(M2), ..., joint21(M21)
```
응 맞아
---

## 5. 목표 속도 어드레스 확인

현재 Goal Velocity 어드레스: **552** (ADDR_PRO_GOAL_VELOCITY)

Dynamixel-P 시리즈 기준:
- 552: Goal Velocity (속도 제어 모드용)
- **560: Profile Velocity (위치 제어 모드에서 속도 프로파일)**

위치 제어 모드(Position Control Mode) 사용 시 Profile Velocity(560) 를 써야
속도 제한이 실제로 적용됩니다. 현재 `torque_read_degree_write.cpp` 에서 552를
사용하여 정상 동작하는지 확인 후 필요하면 560으로 변경해주세요.

**확인 결과:**
```
[ ] 552 (Goal Velocity) 로 속도 제어 동작 확인됨
[ ] 560 (Profile Velocity) 으로 변경 필요
```

→ 560으로 변경 시 코드에서 `ADDR_PRO_GOAL_VELOCITY 552` 를 `560` 으로 수정

---

## 6. 제어 주파수 (`CONTROL_FPS`)

현재 값: `60` Hz (CSV 퍼블리셔 발행 주파수와 동일하게 유지)

실제 CSV 퍼블리셔 FPS와 일치시켜야 비례 속도 계산이 정확합니다.

퍼블리셔 주파수는 120 Hz를 사용하려고 했는데, 가끔 30 Hz를 사용할 수도 있어. 사용하는 주파수에 맞게 매번 제어 주파수를 정확하게 맞춰주면 될까?

**사용할 FPS:**
```
CONTROL_FPS = _______  (CSV 퍼블리셔 CONTROL_FPS 와 동일하게)
```
