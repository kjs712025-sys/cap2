# 안드로이드 앱 개발 — 코딩 에이전트용 프롬프트

아래 내용을 Android Studio의 AI 에이전트(예: Claude, Gemini in Android Studio,
Cursor 등)에 그대로 붙여넣어 사용하세요. 에이전트는 서버 소스 코드에 접근하지
못한다고 가정하고, 이 문서만으로 앱을 완성할 수 있도록 작성돼 있습니다.

---

## 프롬프트 시작 ↓↓↓

너는 안드로이드 앱을 처음부터 만드는 시니어 엔지니어다. "OmniBot"이라는 자율주행
로봇을 원격으로 모니터링·조작하는 안드로이드 앱을 만든다. 이 앱은 이미 존재하는
웹 대시보드(FastAPI 백엔드가 제공)와 기능이 유사한 네이티브 클라이언트다.

### 1. 목표

- 로봇 상태(모드/배터리/근접거리/화재감지 등)를 실시간으로 본다.
- **자율주행(autonomous) ↔ 수동조작(manual)** 모드를 전환한다.
- 수동조작 모드에서 **가상 조이스틱**과 **LLM 텍스트/음성 명령**으로 로봇을 움직인다.
- 카메라 실시간 영상(MJPEG), 라이다 레이더, SLAM 점유격자 지도를 본다.
- 화재 감지 시 알림/사이렌.
- 비상정지 버튼.

### 2. 백엔드 API 명세

**Base URL**: `http://<ROBOT_IP>:8000` — 기본값 `http://192.168.137.97:8000`.
앱 설정 화면에서 IP/포트/토큰을 수정할 수 있어야 한다(DataStore 또는
SharedPreferences 저장, 기본값 위와 같음). 평문 HTTP이므로 network security
config 또는 `usesCleartextTraffic`가 필요하다(2.7 참고).

**응답 봉투** — 모든 JSON 응답은 다음 형태다:
```
성공: { "success": true,  "message": "...", "data": { ... } }
실패: { "success": false, "message": "사람이 읽는 메시지", "error": "머신 코드" }
```
실패도 대부분 HTTP 200으로 온다(토큰 오류만 401). 항상 `success` 필드로 판단하라.

**인증** — 서버가 `ROBOT_API_TOKEN`을 설정한 경우에만 필요.
`Authorization: Bearer <token>` 헤더 또는 `?token=<token>` 쿼리.
`GET /api`, `GET /health`는 토큰 없이 열려 있다. 앱은 토큰을 설정에서
선택적으로 입력받아 모든 요청에 붙인다(빈 값이면 안 붙임).

**CORS**: 서버가 모든 오리진 허용(`*`), credentials 미사용. 네이티브 앱이므로
CORS는 무관하지만 참고.

#### 2.1 서비스 디스커버리

`GET /api` → `data`:
```json
{
  "robot_name": "OmniBot",
  "version": "0.1.0",
  "auth_required": false,
  "streams": {
    "camera_mjpeg": "/camera/stream",
    "navigation_sse": "/navigation/stream",
    "slam_sse": "/slam/stream",
    "fire_sse": "/fire/stream",
    "mqtt": { "base_topic": "robot", "port": 1883, "fire_topic": "robot/fire" }
  },
  "endpoints": { ... }
}
```
앱 시작 시 이걸 호출해 연결 상태를 확인하고 `robot_name`을 헤더에 표시하라.

#### 2.2 상태 / 텔레메트리 (폴링, 2초 간격)

`GET /health` → `data: { status:"ok", uptime_seconds, mode }`
`GET /telemetry` → `data`:
```json
{
  "robot_name": "OmniBot", "mode": "manual|autonomous|idle|error",
  "mission": "ready", "battery": 0.0, "temperature": 0.0,
  "current_speed": [vx, vy, wz], "ai_state": "standby",
  "proximity": 0.42, "safe_direction": "forward",
  "heartbeat": 123, "fire": "clear|detected|<warning text>"
}
```
`GET /diagnostics` → 컴포넌트별(stm32/camera/lidar) 상태. 하드웨어 카드 표시용.
`GET /events?limit=15` → `data: { events: [ {kind, timestamp, ...} ] }`
`GET /alerts` → `data: { alerts: [...] }`

#### 2.3 제어 모드

`GET /mode` → `data: { mode: "manual" | "autonomous" | "error" }`
`POST /mode` body `{ "mode": "auto" }` 또는 `{ "mode": "manual" }`
 → `data: { mode: ... }`

- **autonomous**: 로봇이 스스로 주행. 수동 입력(조이스틱/LLM/음성)은 서버가
  거부한다(`error: "autonomous_active"`).
- **manual**: 조이스틱·LLM·음성 명령 사용 가능.
- 앱은 `/navigation/stream`(2.4)의 `enabled` 필드로 현재 모드를 실시간 추적하고,
  autonomous일 때 수동 조작 UI를 비활성화(회색 처리)하라.

#### 2.4 자율주행 상태 (SSE)

`GET /navigation/stream` — `Content-Type: text/event-stream`, 약 4Hz.
각 이벤트: `data: <JSON>\n\n`. JSON:
```json
{
  "enabled": true,
  "decision": "cruise|avoid|turn_left|turn_right|reverse|halted|idle",
  "reason": "front clear (1.20 m)",
  "velocity": { "vx": 0.15, "vy": 0.0, "wz": 0.0 },
  "sectors": {
    "front":       { "min_distance": 1.2, "count": 40, "source": "lidar|camera" },
    "front_left":  { "min_distance": null, "count": 0, "source": "lidar" },
    "front_right": { ... }, "left": { ... }, "right": { ... }, "rear": { ... }
  },
  "camera": { "available": true, "floor_frac": 0.9, "overrode": { "front": 0.35 } },
  "slam": { "pose": { "x": 0.1, "y": 0.4, "yaw": 1.5 }, "trail_len": 14 },
  "proximity": 0.36,
  "safe_direction": "forward|left|right|backward|stop",
  "scan": [ { "angle_deg": 0.0, "distance_m": 0.9 }, ... ],   // ~180개, 라이다 포인트
  "stm32": { "connected": false, "armed": false, "battery_voltage": null, "last_error": null },
  "fire": { "level": "clear|warn|suppressed|stop", "detections": [ ... ], "timestamp": 0.0 }
}
```
`GET /navigation/state` — 위와 동일한 JSON을 1회성으로. SSE 연결 실패 시 폴백으로
0.7초 간격 폴링하라.

`POST /navigation/start` / `POST /navigation/stop` — `/mode {auto|manual}`의 별칭.
둘 중 하나만 써도 된다(앱은 `/mode`를 권장).

`POST /emergency-stop` — 즉시 정지 + 자율주행 해제 + `error` 모드 래치.
`POST /emergency-stop/clear` — 래치 해제, `manual` 모드로 복귀(정지 상태).
 → `data: { mode: "manual" }`

#### 2.5 수동 조작 (manual 모드에서만)

`POST /motion/command` body `{ "vx": 0.2, "vy": 0.0, "wz": -0.3 }`
 - vx 전진(+)/후진(-) m/s, vy 좌(+)/우(-) m/s, wz 좌회전·CCW(+)/우회전(-) rad/s
 - 한계: |vx|,|vy| ≤ 0.5, |wz| ≤ 1.0 (서버가 클램프하지만 앱도 클램프)
 - 조이스틱을 누르는 동안 ~10Hz로 전송, 손 떼면 즉시 `{0,0,0}` 1회 전송
 - autonomous면 `{success:false, error:"autonomous_active"}` → 사용자에게 토스트

`POST /command/manual` body `{ "text": "앞으로 가" }` — Gemini가 자연어를
**시간제한 속도**로 해석(예: "왼쪽으로 90도 돌아"). 응답 `data.motion`:
```json
{ "action": "move|turn|strafe|stop|none", "vx":0.0, "vy":0.0, "wz":0.0,
  "duration_s": 1.5, "speech": "전진합니다", "transcript": "앞으로 가" }
```
`POST /command/voice` — **원시 오디오 바디**(JSON 아님). `Content-Type: audio/wav`,
바디는 16kHz 모노 16-bit PCM WAV 바이트. 응답은 `/command/manual`과 동일한
`data.motion`(+ `transcript`에 인식된 문장). 빈 바디는 `error: "no_audio"`.

`POST /command/stop` — 진행 중인 수동 모션 취소. `data: { action: "stop", ... }`.

#### 2.6 카메라 / 라이다 / SLAM / 화재

`GET /camera/stream` — **MJPEG** (`multipart/x-mixed-replace; boundary=frame`).
각 파트는 `--frame\r\nContent-Type: image/jpeg\r\n\r\n<JPEG 바이트>\r\n` 형태.
화재 감지 시 프레임에 테두리+박스가 그려져 온다. 무한 스트림.
`POST /camera/analyze` — Gemini 장면 분석. `data: { summary, detections: [...] }`.
`POST /camera/capture` — 서버에 정지영상 저장. `data: { path }`.

`GET /lidar/scan` → `data`:
```json
{ "scan": [ {angle_deg, distance_m}, ... ],
  "obstacles": [ {distance_m, angle_deg, warning}, ... ],
  "safe_direction": "forward", "proximity": 0.42 }
```

`GET /slam/stream` — SSE, 1Hz. `data: <JSON>`:
```json
{
  "resolution_m": 0.05, "size": 200,
  "origin_m": { "x": -5.0, "y": -5.0 },
  "pose": { "x": 0.13, "y": 0.39, "yaw": 1.5 },
  "trail": [ [x, y], ... ],           // 로봇 이동 경로(월드 m)
  "grid_b64": "<base64>",             // size*size 바이트, 0=미탐색 1=자유 2=장애물
  "occupied_cells": 700, "explored_frac": 0.12,
  "scan_points": [ {angle_deg, distance_m, x, y}, ... ],
  "updated_at": 0.0
}
```
`GET /slam/status`, `GET /slam/map` → `data: { pose, resolution_m, size, origin_m,
occupied: [[col,row,prob], ...], trail, scan_points }`
`GET /slam/map/image` — 점유격자 PNG(간편 표시용).

`GET /fire/state` → `data: { level, detections: [ {label, confidence, box:[x1,y1,x2,y2]} ], timestamp }`
`GET /fire/stream` — SSE. 연결 즉시 현재 상태 1회 + 변화 시마다 푸시. 15초마다
`: keepalive` 주석. `level`이 `warn`/`suppressed`/`stop`이면 앱에서 알림+사이렌
(진동 + 짧은 경고음), `clear`로 돌아오면 해제.

#### 2.7 에러 코드 (error 필드)

| 코드 | 의미 | 앱 처리 |
|---|---|---|
| `autonomous_active` | 자율주행 중 수동 명령 시도 | "수동 모드로 전환하세요" 토스트 |
| `emergency_latched` | 비상정지 래치됨 | 비상정지 해제 안내 |
| `bad_mode` | `/mode`에 잘못된 값 | 개발 오류, 무시 |
| `no_audio` | 빈 음성 바디 | "다시 녹음" |
| `camera_unavailable` | 카메라 없음 | placeholder |
| `unauthorized` (HTTP 401) | 토큰 불일치 | 설정 화면으로 유도 |

### 3. 앱 화면 / 기능

단일 액티비티 + Compose Navigation. 하단 탭 또는 스크롤형 대시보드 중 택1
(스크롤형 권장 — 웹 대시보드와 유사).

1. **상단 바** — `robot_name`, 연결 상태(온라인/오프라인), 업타임, 사이렌 토글.
2. **제어 모드 카드** — 큰 버튼 2개: `🕹 수동조작` / `🤖 자율주행`.
   현재 모드 강조. 전환 시 `POST /mode`.
3. **로봇 상태 그리드** — 모드/미션/배터리/온도/속도/AI상태/근접/안전방향/
   하트비트/화재. `/telemetry` 2초 폴링 + SSE 값으로 갱신.
4. **하드웨어 카드** — STM32/카메라/라이다/LLM 온오프라인. `/diagnostics`.
5. **자율주행 패널**
   - **레이더 캔버스**: `/navigation/stream`의 `scan` 포인트를 극좌표로 그리기,
     섹터 여유거리 wedge(카메라 소스는 점선/다른 색), 진행 방향 화살표,
     중앙에 로봇 아이콘. (웹 대시보드의 radar와 동일 개념)
   - 결정/속도/여유거리/안전방향 텍스트.
   - Start/Stop은 제어 모드 카드로 대체.
6. **SLAM 지도 패널** — `/slam/stream`. `grid_b64`를 디코딩해 size×size Bitmap
   으로 그리고(0 투명, 1 옅은 회색, 2 빨강), 그 위에 `trail` 폴리라인과
   `pose` 화살표, `scan_points`를 점으로. 포즈 x/y, 방위, 탐색률 표시.
7. **카메라 패널** — MJPEG 실시간 뷰 + "분석" 버튼(`/camera/analyze` 결과 표시).
8. **모션 컨트롤 패널** (manual 모드에서만 활성)
   - **가상 조이스틱**: 원형 드래그 → (vx, vy). 위=전진(vx+), 오른쪽=우이동(vy-).
   - **회전 슬라이더/다이얼**: wz (-1.0 ~ +1.0), 놓으면 0 복귀.
   - 누르는 동안 100ms마다 `/motion/command`, 놓으면 `{0,0,0}` 1회.
   - 정지 버튼 → `/command/stop`.
9. **음성/텍스트 명령 패널** (manual 모드에서만 활성)
   - 텍스트 입력 + 전송 → `/command/manual`.
   - 🎤 버튼: 누르면 녹음 시작(빨간 펄스), 다시 누르면 종료 후 전송.
     16kHz 모노 WAV로 인코딩해 `/command/voice`에 raw 바디 POST.
     결과의 `transcript`와 `speech`를 표시.
10. **비상정지** — 항상 활성인 큰 빨간 버튼 → `POST /emergency-stop` (즉시 정지 +
    자율주행 해제 + `error` 모드로 래치). 그 옆에 **해제(Clear)** 버튼 →
    `POST /emergency-stop/clear` (안전 확인 다이얼로그 후 호출).
    `data: { mode: "manual" }`을 반환하고 로봇은 정지 상태의 수동 모드로 복귀
    한다(자율주행은 자동 재개되지 않음 — 사용자가 다시 켜야 함).
    `error` 모드 동안에는 조이스틱/LLM/음성이 모두 `emergency_latched`로 거부되므로
    UI에서 "비상정지 해제 필요" 배너를 띄워라.
11. **이벤트/알림 리스트** — `/events`, `/alerts`.
12. **설정 화면** — 로봇 IP, 포트, API 토큰(선택), 폴링 주기.

### 4. 기술 스택

- **언어/UI**: Kotlin + Jetpack Compose (Material 3), 다크 테마 우선.
- **최소 SDK**: 26 (Android 8.0). target/compile: 최신 안정.
- **네트워크**: Retrofit 2 + OkHttp 4 + kotlinx.serialization(또는 Moshi).
- **SSE**: `com.squareup.okhttp3:okhttp-sse` (`EventSource` / `EventSources.createFactory`).
- **MJPEG**: OkHttp 응답 바이트스트림을 직접 파싱(멀티파트 boundary `--frame`,
  각 JPEG를 `BitmapFactory.decodeByteArray`로) → `Canvas`/`Image`에 표시.
  별도 라이브러리 쓰지 말 것(유지보수 안 됨).
- **오디오**: `AudioRecord`(MediaRecorder 아님 — WAV 필요) →
  16000Hz, MONO, PCM_16BIT → 직접 WAV 헤더(44바이트) 붙여 ByteArray.
- **비동기**: Coroutines + Flow. 각 SSE/폴링은 `viewModelScope`의 Flow로,
  화면에 `collectAsStateWithLifecycle`.
- **저장**: DataStore(Preferences)로 설정값.
- **DI**: Hilt (선택, 없어도 됨).
- **아키텍처**: MVVM. `RobotRepository`가 모든 API/스트림을 캡슐화,
  `DashboardViewModel`이 상태 Flow 노출.

### 5. 구현 세부 지침

- **연결 상태**: 마지막 성공 응답 시각을 추적, 3초 이상 무응답이면 "오프라인".
  모든 스트림/폴링은 실패 시 지수 백오프로 재연결(최대 5초). 앱은 죽지 않는다.
- **모드 게이팅**: `enabled == true`면 `currentMode = autonomous`. 조이스틱·회전·
  텍스트·음성 컴포저블에 `enabled = (currentMode == manual)`. autonomous로 바뀌면
  진행 중이던 녹음/조이스틱 드래그를 취소.
- **조이스틱 매핑**: 조이스틱 벡터 (dx, dy) ∈ [-1,1] → `vx = -dy * 0.5`,
  `vy = -dx * 0.5` (화면 위가 전진, 화면 오른쪽이 우측이동=vy 음수).
  회전 다이얼 값 r ∈ [-1,1] → `wz = r * 1.0`. 전송 전 최종 클램프.
- **WAV 인코딩**: `AudioRecord`로 PCM16 버퍼 누적 → RIFF/WAVE 헤더
  (mono, 16000Hz, 16bit) 작성 → `RequestBody.create("audio/wav".toMediaType(), bytes)`.
  0.3초 미만이면 전송하지 않고 "너무 짧음".
- **SLAM 그리드 디코드**: `Base64.decode(grid_b64)` → `ByteArray(size*size)`.
  월드→화면 변환: `sx = (wx - origin_m.x) / resolution_m`, 세로축 뒤집기.
  Bitmap을 size×size로 만들어 픽셀 채우고 canvas에 확대 draw(필터 없이).
- **레이더 그리기**: 캔버스 중심 = 로봇. 각 scan 포인트 `(angle_deg, distance_m)`
  → `r = distance/NAV_MAX(=3.0m) * radius`, `θ = angle_deg`. 화면 위쪽이 전방
  (θ=0). 섹터 wedge는 `sectors[name].min_distance`로. `source=="camera"`면 점선.
- **화재 알림**: `/fire/stream`의 level이 clear가 아니고 사이렌 토글이 켜져 있으면
  진동 패턴 + `ToneGenerator` 경고음 반복 + 상단 배너. `clear` 오면 중지.
  (Web Audio가 아니라 안드로이드 `Vibrator` + `ToneGenerator` 사용)
- **에러 표시**: `success=false`면 `message`를 Snackbar/Toast로. 조작 실패로
  화면이 멈추면 안 됨.
- **수명주기**: 앱이 백그라운드로 가면 모든 SSE/폴링/MJPEG 중지, 복귀 시 재개.

### 6. 비기능 요구사항

- `AndroidManifest.xml`: `INTERNET`, `RECORD_AUDIO`, `VIBRATE` 권한.
  녹음 권한은 런타임 요청.
- **평문 HTTP 허용**: `res/xml/network_security_config.xml`로 사설망 대역
  (`192.168.0.0/16`, `10.0.0.0/8`, `172.16.0.0/12`) cleartext 허용하고
  manifest에 `android:networkSecurityConfig` 연결. (전역 `usesCleartextTraffic`
  보다 이 방식 선호.)
- 세로 모드 기본, 태블릿 대응 레이아웃(넓으면 2열).
- 접근성: 버튼에 contentDescription, 최소 터치 48dp.
- README에 빌드 방법, 로봇 IP 설정법, 권한 설명.

### 7. 산출물

- 컴파일되는 Android Studio 프로젝트(Gradle Kotlin DSL).
- 위 12개 화면/기능 구현.
- `RobotRepository` + `DashboardViewModel` + Compose UI 분리.
- 최소 유닛 테스트: WAV 인코더, SLAM 그리드 디코더, 조이스틱→속도 매핑,
  응답 봉투 파서.
- 하드코딩 금지: 로봇 주소/토큰은 전부 설정에서.
- 먼저 프로젝트 구조와 의존성 계획을 제시하고, 그 다음 구현하라.

## 프롬프트 끝 ↑↑↑

---

## 사용 팁

- 에이전트가 한 번에 다 못 만들면 **화면 단위로 쪼개서** 시키세요
  (① 프로젝트 뼈대 + 설정 + Repository, ② 텔레메트리/모드/상태 카드,
   ③ 레이더 + SLAM 캔버스, ④ MJPEG 카메라, ⑤ 조이스틱, ⑥ 음성/텍스트 명령,
   ⑦ 화재 알림/비상정지/이벤트).
- 실제 로봇 서버는 `http://192.168.137.97:8000` — 같은 Wi-Fi(`meka`)에서
  `GET /api`가 응답하면 연결 정상.
- 서버가 켜져 있으면 `http://192.168.137.97:8000/app/` 를 브라우저로 열어
  참고 UI(웹 대시보드)를 볼 수 있습니다.
