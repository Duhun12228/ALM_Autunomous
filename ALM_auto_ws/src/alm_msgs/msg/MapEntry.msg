# 맵 하나 = maps/<name>/ 디렉터리 하나.
# manifest.yaml 이 있는 폴더만 맵으로 인정한다 (실험 잔재 폴더 배제).

string name                  # 폴더 이름 = 맵 ID
string label                 # manifest.yaml 의 사람이 읽는 이름
string path                  # 절대 경로
string created               # manifest.yaml 의 생성 시각 문자열
string notes                 # manifest.yaml 의 비고

bool active                  # maps/active.yaml 이 가리키는 맵인가
bool complete                # cloud/grid/fpfh 가 모두 present && !stale

alm_msgs/MapAsset[] assets   # 항상 3개 (cloud, grid, fpfh) — 없으면 present=false
