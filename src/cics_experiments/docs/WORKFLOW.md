# CICS workflow and preservation

## 최종 구현 범위

공통 방법은 micro-yaw directional initiation and body-yaw recovery다. 휠체어는 사전 설정형(B3H), 환자이승은 최소 응답 종료형(P)을 플랫폼 내 모든 조건에 고정한다. CICS의 P는 IJCAS 경로계획 P와 다른 알고리즘이다.

원본 174개 SDK/분석 스크립트 중 최종 두 플랫폼 실험과 지표 산출의 정적 의존성을 추적하여 SDK 14개, 분석 7개를 활성 패키지로 옮겼다. 원본 파일명은 추적성을 위해 유지한다. 내부 휠체어 scheduler 파일명의 final9는 실제 seven-condition 행렬이라는 원본의 명명 관례이며 조건을 새로 9개로 늘린 것이 아니다.

경로 관련 모듈 전역 상수·입출력 위치만 바꾸었다. 원본 제어·안전·지표 함수와 클래스는 AST 비교로 동일성을 검사한다. `config/source_manifest.json`은 원본과 활성 소스의 SHA-256 및 대응 관계다.

## 실행

표준 진입점은 다음과 같다.

```bash
source install/setup.bash
ros2 run cics_experiments check
ros2 run cics_experiments experiment --list
ros2 run cics_experiments experiment --platform wheelchair --condition 90p0_center --method proposed
ros2 run cics_experiments reproduce_results
```

`check`와 선택기의 기본 stage는 SDK/Timeline을 시작하지 않는다. `prepare`는 USD를 열어 원래 준비 설정을 Session Layer에 적용할 뿐 Play나 동적 주행을 하지 않는다. `run --allow-motion`은 새 source·새 정적 접촉 검사 후 원본 루틴의 3회를 한 번 실행한다. 전체 240회를 자동 재시험하거나 좋은 결과만 선택하지 않는다.

ROS/system Python과 SDK Python을 별도 프로세스로 분리한다. SDK 프로세스는 ROS Python/공유 라이브러리 경로를 받지 않으며, 원래 full GUI 경험의 fixed timeline stepping을 유지한다. 다른 PC에서는 `--sdk-python /path/to/Isaac-Python-3.11`을 지정한다.

새 실행은 원본을 수정하지 않도록 별도 `log/cics/experiments` 아래의 출력 경로로 분리한다. 새 static 결과가 검사를 통과하지 못하면 동적 실행을 시작하지 않는다. 원본 정적 coroutine의 local `passed` boolean을 반환하도록 메모리 내 반환 계측만 추가하며 원래 판정식이나 임계값은 바꾸지 않는다.

## 원본 재계산

휠체어 35조합과 환자이승 45조합의 raw trace에서 원래 거리창·jerk 필터·경로 보간·반복 분산 정의로 지표를 계산한다. 기존 분석 함수를 그대로 사용한다. 재생성한 플랫폼별 표와 통합 표 네 개, 총 여섯 표의 라벨·행 수·수치를 원본 CSV와 대조한다.

모든 출력은 새로운 `log/cics/reproduced/<timestamp>`에 저장한다. 원본 논문 표·그림·raw 기록을 덮어쓰지 않는다. `--figures`를 주면 원본 제작 함수로 그림도 새 폴더에 생성한다. 수치 재계산은 새 물리 주행을 의미하지 않는다.

## 정리와 복구

- `archive/original/studies`: 원본 실험 로그, 이전 결과, 문서, 발표·논문 자료
- `archive/original/isaac`, `tools`, `scripts`: 원본 SDK/분석·별칭과 이전 개발 코드
- `archive/original/src`: 이전 ROS 초기 조향 prototype 및 최종 모델이 참조하지 않는 옛 모델
- `archive/original/build`, `install`, `log`: 이름 변경 전 생성물
- `archive/migration/original_manifest.json`: 정리 전 원본 파일 hash, symlink, Git HEAD와 수정 상태
- `archive/migration/verification.json`: 이동·경로·원본 보존 검증 결과
- `archive/migration/commissioning.json`: 빌드·시험·대표 실주행 및 원본 수치 대조의 최종 기록

기존 README의 configuration-aware initial steering / Nav2 prototype은 최종 CICS 미세-yaw 제어와 다르므로 활성 `src`에서 분리했다. 해당 사용자 수정과 원본 파일은 삭제하지 않고 `archive/original/src/patient_initial_stability`에 보존했다. 필요하면 별도 작업공간에서 복원할 수 있다.

보관 자료는 불변 원본이므로 과거 경로 문자열도 고치지 않았다. 활성 코드의 `paths.resolve_path`가 현재 `CICS_ws` 내부의 코드·원본 자료·새 출력 위치를 구분한다. 옛 workspace가 존재해야 하는 fallback이나 옛 이름의 symlink는 만들지 않는다.

`archive/COLCON_IGNORE`로 이전 package.xml이 중복 빌드되지 않게 했으며 archive·실험 로그·빌드 산출물은 Git에서 제외한다. 선행 저장소를 가리키던 원래 `.git`은 `archive/migration/original_git`에 그대로 옮겨 보존한다. 활성 `.git`은 CICS 공개 코드만 포함한 별도 이력이며 기존 저장소 커밋을 덮어쓰지 않는다. `check --full`은 보관한 원래 Git의 inode와 HEAD까지 대조한다. 이 부분은 로컬 보존 검사이며 다른 PC의 clone에는 해당 원본 Git이 없다.

기존 폴더를 열어 둔 VS Code 창은 새 `CICS_ws` 폴더로 다시 열어야 한다. C++ 인덱서가 옛 경로에 재생성한 `.vscode` 데이터베이스는 실행 의존성이 아니며, workspace 경로 호환용 symlink로 사용하지 않는다.

## 검증 범위

정상 빌드, cwd와 무관한 표준 소싱, 80개 선택 recipe와 원본 call 인자 일치, 원본 함수/클래스 AST, 원본 raw 기록의 지표 재계산, USD 외부 참조와 새 SDK 장면 준비를 각각 확인한다. 새 전체 비교 행렬을 다시 실행했다거나 실제 로봇 안전·성능을 보증한 것으로 표현하지 않는다. 실제 검증 결과는 `log/cics`와 `archive/migration`의 기록을 확인한다.

최종 검증에서는 3개 패키지 빌드, 106개 자동 검사, 원본 240회 기록으로부터 여섯 결과표의 수치 일치를 확인했다. 환자이승 `265center/P`와 휠체어 `90p0_center/B3H`의 새 실행은 각각 3/3 목표 도달·캐스터 전환을 기록하고 새 정적 검사 및 실제 Pause 반영을 통과했다. 이 대표 조건들의 원본 trace 공통 열 값과 원본 지표는 일치했다. 추가 진단 열이 있어 CSV 파일 전체의 byte 동일성을 뜻하지는 않는다.

초기 검증에서 원본 coroutine의 비동기 Pause 요청을 같은 이벤트 순환에서 즉시 판정한 실행기 오류가 있었다. 원본 Pause 요청이 적용될 때까지 확인하는 실행 포장과 완료 기록 검사를 수정했으며, 제어 함수·판정식·임계값은 변경하지 않았다. 초기 오류 기록과 후속 검증 기록은 모두 별도로 보존하고 원본 논문 통계에 합치지 않았다.
