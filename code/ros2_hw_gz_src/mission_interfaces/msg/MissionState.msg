# /mission/state (husky_mission 발행, latched): 임무 단계와 지금 세트의 탐색 구역
string PHASE_IDLE=IDLE          # 임무 요청(RunMission) 대기
string PHASE_TRANSIT=TRANSIT    # Husky 가 세트 주차점으로 이동, A1 은 뒤따름
string PHASE_EXPLORE=EXPLORE    # A1 이 자기 구역 탐색
string PHASE_REGROUP=REGROUP    # 탐색 시간 초과 -> 남은 A1 도 탐색 시작 위치로
string PHASE_DONE=DONE          # 모든 세트 끝 (또는 임무 취소)

string phase
int32 set_index                 # 지금 세트 번호 (config 의 sets 순서, 0 부터)
string set_name                 # set1 / set2 / set3 / set4 / common
mission_interfaces/Zone[] zones # EXPLORE·REGROUP 때만 채움 (그 밖에는 빈 배열)
bool comm                       # Husky-A1 통신 (false 면 A1 은 탐색 시작 위치로 복귀)
