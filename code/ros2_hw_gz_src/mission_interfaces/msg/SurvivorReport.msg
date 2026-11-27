# /husky/survivor_report (각 A1 의 survivor_detector -> husky_mission): 생존자 한 명 발견 보고.
# 통신이 끊겨 있으면 A1 이 모아 두었다가 연결되면 보낸다.
std_msgs/Header header          # stamp = 보낸 시각, frame_id = map (그 로봇 TF 트리의 map, 원점 = 그 로봇 스폰, 방향 = 월드)
string robot                    # a1_1 / a1_2
uint32 id                       # 그 로봇 안에서의 생존자 번호
geometry_msgs/Point position    # 생존자 위치 (header.frame_id 좌표, m)
float32 score                   # 인식 점수 0~1
float32 delay                   # 발견 -> 보고까지 걸린 시뮬 초 (통신 끊김 동안 쌓이면 커짐)
