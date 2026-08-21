# 맵 폴더 안의 자산 하나. map_manager 가 파일 '헤더만' 읽어 채운다
# (cloud.pcd 는 52 MB — 통째로 읽으면 안 된다).

# 자산 종류
string KIND_CLOUD=cloud      # cloud.pcd            — FAST-LIO /map_save 출력
string KIND_GRID=grid        # grid.pgm + grid.yaml — pcd2pgm 출력
string KIND_FPFH=fpfh        # fpfh_map.meta + _points/_normals/_fpfh.pcd

string kind
bool present                 # 파일이 존재하는가
bool stale                   # 존재하지만 부모 cloud.pcd 와 짝이 맞지 않는가
uint64 size_bytes            # 대표 파일 크기 (grid=pgm, fpfh=세 pcd 합)
float64 modified_epoch       # 대표 파일 mtime [s]. 0 = 없음

# 사람이 읽는 요약. UI 가 그대로 표시한다.
#   cloud: "1,623,894 pts"   grid: "1014 x 671 @0.05 m"   fpfh: "1,414 features"
string detail

# stale 인 이유. 정상이면 빈 문자열.
# 예: "cloud.pcd 가 DB보다 최신입니다 — fpfh_map_builder 재실행 필요"
string issue
