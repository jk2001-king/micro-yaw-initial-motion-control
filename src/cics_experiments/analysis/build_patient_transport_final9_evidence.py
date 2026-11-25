"""Build reproducible evidence for the patient-transport 9-condition matrix."""

from cics_experiments.paths import workspace_root, resolve_path

import csv
import statistics
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import compare_patient_transport_cics26_lateral_fullmatrix as comparison


ROOT = workspace_root()
RESULT_ROOT = resolve_path('studies/CICS26/results/patient_transport_lab/natural_flip_pilot')
OUT = resolve_path('studies/CICS26/paper_assets/patient_transport/final9')
DOC = resolve_path('studies/CICS26/docs/patient_transport_lateral_final9_results.md')
METHOD_LABELS = {
    "B0": "Immediate",
    "B1": "Fixed low-speed",
    "B2": "Acceleration ramp",
    "P": "Response-step + hold",
    "B3H": "Fixed micro-yaw + hold",
}
METRICS = ("lat", "yaw", "jerk", "time", "spread")


def median(values):
    return statistics.median(values)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    results = {
        (condition, method): comparison.metrics(condition, "frontpos", method)
        for condition in comparison.CONDITIONS
        for method in comparison.METHODS
    }

    rows = []
    for condition in comparison.CONDITIONS:
        for method in comparison.METHODS:
            value = results[condition, method]
            rows.append({
                "condition": condition,
                "method": method,
                "method_label": METHOD_LABELS[method],
                "reached_count": value["reached"],
                "flip_count": value["flip"],
                "median_max_lateral_mm": value["lat"],
                "median_max_yaw_deg": value["yaw"],
                "median_peak_jerk_m_s3": value["jerk"],
                "median_travel_time_s": value["time"],
                "trajectory_spread_mm": value["spread"],
            })
    fields = list(rows[0])
    with (OUT / "patient_transport_final9_metrics.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    aggregate = {}
    for method in comparison.METHODS:
        aggregate[method] = {
            key: median(results[condition, method][key]
                        for condition in comparison.CONDITIONS)
            for key in METRICS
        }
        aggregate[method]["flip"] = sum(
            results[condition, method]["flip"]
            for condition in comparison.CONDITIONS
        )
        aggregate[method]["wins"] = {
            key: sum(
                method == min(
                    comparison.METHODS,
                    key=lambda candidate: results[condition, candidate][key],
                )
                for condition in comparison.CONDITIONS
            )
            for key in METRICS
        }

    reductions = {}
    for baseline in ("B0", "B1", "B2", "P"):
        reductions[baseline] = {}
        for key in METRICS:
            values = [
                100.0 * (1.0 - results[condition, "B3H"][key]
                         / results[condition, baseline][key])
                for condition in comparison.CONDITIONS
            ]
            reductions[baseline][key] = {
                "median": median(values),
                "better": sum(value > 0.0 for value in values),
            }

    p_rows = []
    for condition in comparison.CONDITIONS:
        path = (RESULT_ROOT / f"cics26_pt_{condition}_lateral_lowbed_frontpos_P_v5"
                / "summary.csv")
        p_rows.extend(read_csv(path))
    response_count = sum(row["adaptive_response_reached"] == "True" for row in p_rows)
    selected_rates = sorted({
        float(row["adaptive_selected_rate_rad_s"])
        for row in p_rows if row["adaptive_selected_rate_rad_s"]
    })

    table_lines = []
    for method in comparison.METHODS:
        item = aggregate[method]
        table_lines.append(
            f'| {METHOD_LABELS[method]} | {item["lat"]:.3f} | {item["yaw"]:.3f} | '
            f'{item["jerk"]:.3f} | {item["time"]:.3f} | {item["spread"]:.3f} | '
            f'{item["flip"]}/27 | {item["wins"]["lat"]} | '
            f'{item["wins"]["yaw"]} | {item["wins"]["spread"]} |'
        )

    b2 = reductions["B2"]
    b0 = reductions["B0"]
    p = reductions["P"]
    text = f"""# 환자이승로봇 9조건 최종 결과

Isaac Sim 저상 구조물 삽입-정지-이탈 시나리오에서 3개 총질량
(`240/265/290 kg`)과 3개 base CoM 위치(후방/중앙/전방)를 조합한 9조건에
대해 5방법을 각 3회 기록했다. 총 135회 모두 0.70 m 이탈 목표에 도달했다.
질량과 CoM은 실측 환자 모델이 아닌 시뮬레이션 민감도 조건이다.

## 9조건 중앙값과 조건별 최저 횟수

| 방법 | 횡편차 mm | yaw ° | jerk m/s³ | 시간 s | 분산 mm | 뒤집힘 | 횡 1위 | yaw 1위 | 분산 1위 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(table_lines)}

## 핵심 결과

- B3H는 횡편차에서 9조건 중 8조건 1위였고 전체 중앙값은
  `{aggregate['B3H']['lat']:.3f} mm`다. B2 대비 횡편차 중앙 감소율은
  `{b2['lat']['median']:.1f}%`(9/9 개선), 반복 분산 감소율은
  `{b2['spread']['median']:.1f}%`(9/9 개선)다.
- B3H의 yaw는 B2보다 5/9조건에서 작고 중앙 감소율은
  `{b2['yaw']['median']:.1f}%`였다. 반면 P는 yaw에서 9/9조건 1위로,
  전체 중앙값 `{aggregate['P']['yaw']:.3f}°`를 기록했다.
- B3H의 jerk는 B2보다 9/9조건에서 컸으며 중앙 증가율은
  `{-b2['jerk']['median']:.1f}%`다. B0보다는 9/9조건에서 낮았고 중앙
  감소율은 `{b0['jerk']['median']:.1f}%`다.
- B3H와 P는 모두 뒤집힘 완료 `27/27`이었다. B0/B1/B2는 각각
  `{aggregate['B0']['flip']}/27`, `{aggregate['B1']['flip']}/27`,
  `{aggregate['B2']['flip']}/27`이다.
- P 대비 B3H는 횡편차가 8/9조건에서 작았지만, yaw는 0/9조건,
  jerk는 {p['jerk']['better']}/9조건, 반복 분산은
  {p['spread']['better']}/9조건에서만 작았다. 따라서 B3H를 모든 지표의
  보편적 최적법으로 주장하지 않는다.

## 응답 기반 P의 작동 한계

P는 응답을 `{response_count}/27`에서 검출했으나 선택 yaw-rate는 전 시행에서
첫 단계 `{selected_rates[0]:.9f} rad/s`였다. 임계값을 넘기기 위한 단계 증가가
실제로 발생하지 않았으므로, 본 결과는 **하중에 따른 단계 증가 적응성**을
증명하지 않는다. 관측된 P 성능은 사실상 저진폭 응답검출 후 yaw hold의
성능으로 해석한다.

## 논문 서술 범위

B3H는 하중·CoM 변화 전반에서 가속도 램프 대비 횡편차와 반복 분산을 크게
감소시키고 뒤집힘 완료를 유지한 고정 미세-yaw 방향결정 기법으로 제시할 수
있다. 다만 P가 yaw와 일부 jerk/분산 지표에서 더 우수하고, B3H의 jerk가
램프 기준군보다 증가했으므로 결과를 다목적 상충관계로 서술한다. B1은 목표
속도가 절반이므로 시간과 jerk를 동일 속도 제어법의 직접 비교로 해석하지 않는다.

재현 분석: `python3 studies/CICS26/scripts/build_patient_transport_final9_evidence.py`
원자료: `studies/CICS26/results/patient_transport_lab/natural_flip_pilot/`
"""
    DOC.write_text(text, encoding="utf-8")
    print("saved:", OUT / "patient_transport_final9_metrics.csv")
    print("saved:", DOC)


if __name__ == "__main__":
    main()
