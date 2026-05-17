#include "ImgPreprocessor/ImgPreprocessor.hpp"
#include "Common/Line.h"
#include "ConfigSettings/ConfigSettings.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
using namespace std;

// #define SHOW_VIDEO
// #define debug_log
namespace LCODE {

ImgPreprocessor::ImgPreprocessor(ConfigSettings &config) : lineProcessor_(config) {

  this->ROI_width_dots  = config.getROI_width_dot();  /// ROI 영역의 가로 점 갯수
  this->ROI_height_dots = config.getROI_height_dot(); /// ROI 영역의 세로 점 갯수
  // this->required_baseline_cnt = 계산 공식 - (ROI_width_dots - 1) / 6 + 1;

  this->required_baseline_cnt = 3;

  this->expansion_point_num = (ROI_height_dots - 1) / 6;

  int ROI_size = 800; // ROI 영역 size

  this->ROI_width_size  = ROI_size * (static_cast<float>(ROI_width_dots) / (ROI_height_dots + ROI_width_dots));
  this->ROI_height_size = ROI_size * (static_cast<float>(ROI_height_dots) / (ROI_height_dots + ROI_width_dots));

  params_.minThreshold  = config.minThreshold;  // opencv default : 50
  params_.maxThreshold  = config.maxThreshold;  // opencv default : 220
  params_.thresholdStep = config.thresholdStep; // opencv default : 20

  params_.filterByArea        = config.filterByArea;
  params_.minArea             = config.minArea; // opencv default : 25
  params_.maxArea             = config.maxArea; // opencv default : 5000
  params_.filterByCircularity = config.filterByCircularity;
  params_.minCircularity      = config.min_circularity;
  params_.maxCircularity      = config.max_circularity;
  params_.filterByConvexity   = config.filterByConvexity;
  params_.minConvexity        = config.min_convexity;
  params_.maxConvexity        = config.max_convexity;

  params_.filterByInertia  = config.filterByInertia;
  params_.minInertiaRatio  = config.min_inertia;
  params_.maxInertiaRatio  = config.max_inertia;
  params_.minRepeatability = config.min_repeatability;
};

void ImgPreprocessor::drawDetectedDots(cv::Mat &img, vector<cv::KeyPoint> &keypoints) {
  if (img.type() == CV_8UC1) {
    cvtColor(img, img, COLOR_GRAY2BGR);
  }
  cv::drawKeypoints(img, keypoints, img, cv::Scalar(0, 255, 0));
#ifdef SHOW_VIDEO
  imshow("detected_dots", img);
#endif
};

void ImgPreprocessor::eraseDetectedDots(cv::Mat &img, vector<cv::KeyPoint> &dots) {
  for (const auto &dot : dots) {
    // dot.size + 0.5 => 기존 dot size보다 살짝 더 크게 지우도록 해서, 선 검출에 방해가 되는 점의 흔적을 최대한
    // 제거하려는 의도
    circle(img, dot.pt, static_cast<int>(dot.size + 0.5), {0}, -1);
  }
#ifdef SHOW_VIDEO
  imshow("after eraseDetectedDots, binarized", img);
#endif
  this->warp_source_orient = img.clone();
}

auto ImgPreprocessor::binarizeImage(const cv::Mat &img) -> cv::Mat {
  cv::Mat       output;
  constexpr int mode = 0;
  if (mode == 0) {
    double k                  = 0.07; // 보통 -0.2 ~ -0.5 사이. 작을수록 더 많은 점을 잡습니다.
    int    blockSize          = 31;   // 점의 크기보다 커야 합니다.
    int    binarizationMethod = cv::ximgproc::BINARIZATION_SAUVOLA;
    // Niblack 적용
    // 결과는 검은 배경에 흰 점(BINARY) 또는 반대(BINARY_INV)로 설정 가능합니다.
    cv::ximgproc::niBlackThreshold(img, output, 255, cv::THRESH_BINARY_INV, blockSize, k, binarizationMethod);
  } else {
    constexpr int maxValue       = 255;
    constexpr int adaptiveMethod = cv::ADAPTIVE_THRESH_GAUSSIAN_C;
    constexpr int thresholdType  = cv::THRESH_BINARY_INV;
    constexpr int blockSize      = 15; // 11~21 normally  , 홀수
    int           C = 8; //-10~ 10 normally 가중평균에서 빼는 상수 .. -> threahold 조정역할 클수록 이미지 어두움
    cv::adaptiveThreshold(img, output, maxValue, adaptiveMethod, thresholdType, blockSize, C);
  }

  return output;
}

auto ImgPreprocessor::findClosestDotToCP() -> bool {

  // 1. 중심점 계산
  this->img_cross_point = cv::Point2f(src.cols * 0.5f, src.rows * 0.5f);

  const float         dist_tol_sq = 50.0f * 50.0f; // 허용 범위의 제곱
  float               min_dist_sq = std::numeric_limits<float>::max();
  const cv::KeyPoint *best_dot    = nullptr;

  for (const auto &dot : all_detected_dots_) {
    // 2. 사각형 범위(Bounding Box)로 1차 필터링 (매우 빠름)
    float dx = dot.pt.x - img_cross_point.x;
    if (std::abs(dx) > 50.0f)
      continue;

    float dy = dot.pt.y - img_cross_point.y;
    if (std::abs(dy) > 50.0f)
      continue;

    // 3. 제곱 거리를 이용한 비교 (sqrt 생략으로 성능 향상)
    float current_dist_sq = dx * dx + dy * dy;

    if (current_dist_sq < min_dist_sq && current_dist_sq <= dist_tol_sq) {
      min_dist_sq = current_dist_sq;
      best_dot    = &dot;
    }
  }
  // 4. 결과 처리
  if (best_dot == nullptr) {
    return false;
  }
  this->closest_dot_to_cross_point = *best_dot;
  return true;
}

void ImgPreprocessor::orderBaseLinesByClosestDot() {
  // 점과 선분 사이 거리 , 해당 선분을 묶어 저장
  std::vector<std::pair<std::vector<Line<int>>, double>> line_to_dot_distances;

  for (const auto &cluster : clustered_lines) {
    for (const auto &line : cluster) {
      double dist = distanceFromPointToLine(closest_dot_to_cross_point.pt, line.pt1, line.pt2);
      line_to_dot_distances.push_back({cluster, dist});
    }
  }

  // 거리 기준으로 선들을 오름차순 정렬
  sort(line_to_dot_distances.begin(), line_to_dot_distances.end(),
       [](const pair<std::vector<Line<int>>, double> &a, const pair<std::vector<Line<int>>, double> &b) {
         return a.second < b.second;
       });

  for (int i = 0; i < line_to_dot_distances.size(); i++) {
    this->closestLines.push_back(line_to_dot_distances[i].first);
  }

  if (this->closestLines.size() >= 2) {
    auto get_side_vec = [&](const std::vector<Line<int>> &cluster) {
      // 선의 중심점과 점(Dot) 사이의 벡터 반환
      cv::Point2f center((cluster[0].pt1.x + cluster[0].pt2.x) / 2.0f, (cluster[0].pt1.y + cluster[0].pt2.y) / 2.0f);
      return center - closest_dot_to_cross_point.pt;
    };

    // 거리로 나열한 두번째, 세번째 선이 같은 방향 벡터면 , [0]으로 할당된 선분을 [1]과 교체함
    // closestLines[0]으로 할당되는 선은 항상 중앙에 있는 선분이 되어야하기 때문
    auto vec_1 = get_side_vec(this->closestLines[1]);
    auto vec_2 = get_side_vec(this->closestLines[2]);

    float dot_product = vec_1.x * vec_2.x + vec_1.y * vec_2.y;

    if (dot_product > 0) {
      std::swap(this->closestLines[0], this->closestLines[1]);
    }
  }
}

auto ImgPreprocessor::distanceFromPointToLine(cv::Point pt, cv::Point lineStart, cv::Point lineEnd) -> double {
  double numer = abs((lineEnd.y - lineStart.y) * pt.x - (lineEnd.x - lineStart.x) * pt.y + lineEnd.x * lineStart.y -
                     lineEnd.y * lineStart.x);
  double denom = sqrt(pow(lineEnd.y - lineStart.y, 2) + pow(lineEnd.x - lineStart.x, 2));
  return numer / denom;
}

auto ImgPreprocessor::findValidLinesByDotsLocation() -> bool {
  if (all_detected_dots_.empty() || closestLines.empty()) {
    return false;
  }

  std::vector<std::vector<Line<int>>> validated_lines;

  // 1. 모든 루프에서 고정된 상수 및 제곱값 미리 계산
  const double tolerance     = closest_dot_to_cross_point.size * 1.2;
  const double tolerance_sq  = tolerance * tolerance;
  const int    required_dots = (expansion_point_num * 2) + 1;

  for (const auto &line_group : closestLines) {
    std::vector<Line<int>> valid_line_group;

    for (const auto &lin : line_group) {
      // 2. 선분(lin) 고유의 수식 값들을 '점 루프' 진입 전에 딱 한 번만 계산 (★핵심 최적화)
      const double dx = static_cast<double>(lin.pt2.x - lin.pt1.x);
      const double dy = static_cast<double>(lin.pt2.y - lin.pt1.y);

      const double denominator_sq    = (dy * dy) + (dx * dx);
      const double max_dist_sq_denom = tolerance_sq * denominator_sq;
      const double line_const = static_cast<double>(lin.pt2.x) * lin.pt1.y - static_cast<double>(lin.pt2.y) * lin.pt1.x;

      int cnt = 0;

      // 3. 내부 루프에서는 최소한의 연산만 수행
      for (const auto &dot : all_detected_dots_) {
        const double numerator = (dy * dot.pt.x) - (dx * dot.pt.y) + line_const;

        if ((numerator * numerator) <= max_dist_sq_denom) {
          cnt++;
          if (cnt >= required_dots) {
            break; // 필요한 개수를 채우면 즉시 점 루프 완전히 탈출
          }
        }
      }

      // 조건에 맞는 선만 현재 그룹에 추가
      if (cnt >= required_dots) {
        valid_line_group.push_back(lin);
      }
    }

    // 그룹 내에 유효한 선이 하나라도 있으면 최종 결과에 추가
    if (!valid_line_group.empty()) {
      // std::move를 사용하여 벡터 복사 오버헤드 방지
      validated_lines.push_back(std::move(valid_line_group));
    }
  }

  closestLines = std::move(validated_lines);
  return true;
}

auto ImgPreprocessor::findValidLinesByDotsNum() -> bool {
  // 1. 최소 기준선 개수 확보 체크
  if (closestLines.size() < this->required_baseline_cnt) {
    LBS_error_code = findValidLines_error;
    return false;
  }

  // 매 루프마다 계산하지 않도록 상수를 바깥으로 선언
  const double dist_tolerance         = this->closest_dot_to_cross_point.size * 1.2;
  const double dist_tolerance_sq      = dist_tolerance * dist_tolerance;
  const float  comp_dot_num_tolerance = 2.5f; //

  // 잘못된 선을 제거하고 최대 2번까지 재검사(Retry) 수행
  for (int retry = 0; retry < 2; retry++) {
    // 선이 지워진 후 다음 루프에서 개수 미달 시 바로 실패 처리
    if (closestLines.size() < this->required_baseline_cnt) {
      return false;
    }

    // 각 선 위에 매칭된 점들을 모아둘 벡터 공간
    std::vector<std::vector<cv::KeyPoint>> dots_on_line(this->required_baseline_cnt);

    // [최적화] 선분 고유의 방정식 상수들을 점 루프 진입 전에 미리 캐싱
    struct LineCache {
      double dx, dy, line_const, max_dist_sq_denom;
    };
    std::vector<LineCache> line_caches;
    line_caches.reserve(this->required_baseline_cnt);

    for (int k = 0; k < this->required_baseline_cnt; k++) {
      const auto &ln             = this->closestLines[k][0];
      double      dx             = static_cast<double>(ln.pt2.x - ln.pt1.x);
      double      dy             = static_cast<double>(ln.pt2.y - ln.pt1.y);
      double      denominator_sq = (dy * dy) + (dx * dx);
      double      line_const     = static_cast<double>(ln.pt2.x) * ln.pt1.y - static_cast<double>(ln.pt2.y) * ln.pt1.x;

      line_caches.push_back({dx, dy, line_const, dist_tolerance_sq * denominator_sq});
    }

    // [최적화] 중복 연산 없는 고속 거리 검사 및 점 분배
    for (const auto &key : all_detected_dots_) {
      for (int k = 0; k < this->required_baseline_cnt; k++) {
        const auto &cache     = line_caches[k];
        double      numerator = (cache.dy * key.pt.x) - (cache.dx * key.pt.y) + cache.line_const;

        if ((numerator * numerator) <= cache.max_dist_sq_denom) {
          dots_on_line[k].push_back(key);
        }
      }
    }

    // 다른 선들에 비해 점이 너무 많이 찍힌 (1.5배 이상) 이상치 선 찾아내기
    bool line_erased = false;
    for (int l = 0; l < this->required_baseline_cnt; l++) {
      float comp_cnt = static_cast<float>(dots_on_line[l].size());

      for (int j = 0; j < this->required_baseline_cnt; j++) {
        if (l == j)
          continue; // 자기 자신과의 비교는 제외

        float target_limit = static_cast<float>(dots_on_line[j].size()) * comp_dot_num_tolerance;

        // l번째 선의 점 개수가 다른 선(j)보다 1.5배 이상 많다면 문제 있는 선으로 판단
        if (comp_cnt >= target_limit) {
          this->closestLines.erase(this->closestLines.begin() + l); // 오타 수정: l번째 선 제거
          line_erased = true;
          break;
        }
      }
      if (line_erased)
        break;
    }

    // 선을 지웠다면, 남은 선들로 처음부터 '다시 검사(Retry)' 진행하기 위해 continue
    if (line_erased) {
      continue;
    }

    // 모든 선의 점 개수가 정상 범주라면 결과를 저장하고 최종 성공 리턴
    nth_closest_keypoints_on_baseline = std::move(dots_on_line);
    return true;
  }

  return false;
}
auto ImgPreprocessor::validateClosestBaseLinesByDirection() -> bool {
  // 1. 선분 데이터 추출 (인덱스 1과 2만 콕 집어서 사용)
  // (만약의 에러를 대비해 사이즈 체크 안전장치만 살짝 넣었습니다)
  if (closestLines.size() <= 2 || closestLines[1].empty() || closestLines[2].empty()) {
    return false;
  }

  // 바깥 라인
  auto line1 = closestLines[1][0];
  auto line2 = closestLines[2][0];

  cv::Point2f CP       = {static_cast<float>(img_cross_point.x), static_cast<float>(img_cross_point.y)};
  float       dot_size = closest_dot_to_cross_point.size;

  // 2. CP 기준으로 두 선에 대한 방향 벡터와 거리 계산
  cv::Point2f closest_pt1 = getClosestPointOnLine(line1.pt1, line1.pt2, CP);
  cv::Point2f closest_pt2 = getClosestPointOnLine(line2.pt1, line2.pt2, CP);

  cv::Point2f dir1 = closest_pt1 - CP;
  cv::Point2f dir2 = closest_pt2 - CP;

  float dist1 = static_cast<float>(cv::norm(dir1));
  float dist2 = static_cast<float>(cv::norm(dir2));

  // 점이 선 위에 완벽하게 겹친 경우
  if (dist1 < 1e-5 || dist2 < 1e-5)
    return true;

  // 3. 두 방향 벡터 사이의 각도 차이(내각) 계산
  double angle1 = std::atan2(dir1.y, dir1.x);
  double angle2 = std::atan2(dir2.y, dir2.x);

  double angle_diff = std::abs(angle1 - angle2);
  if (angle_diff > CV_PI) {
    angle_diff = 2.0 * CV_PI - angle_diff; // 180도 넘어가는 겉각 대신 안쪽 내각 사용
  }

  // 4. 조건 판별 (30도 이내면 한쪽으로 쏠린 것)
  const double SAME_DIR_THRESHOLD = 30.0 * CV_PI / 180.0;

  if (angle_diff <= SAME_DIR_THRESHOLD) {
    // 같은 방향으로 쏠렸지만, 둘 중 가장 가까운 선이 허용치 이내라면 true
    return (std::min(dist1, dist2) <= dot_size);
  }
#ifdef SHOW_VIDEO
  cv::Mat test = src.clone(); // 원본 이미지 src를 복제하여 시각화에 사용

  // 화살표 그리기 (CP에서 각 선의 가장 가까운 점으로)
  // Green arrow for dir1, from CP to closest_pt1
  cv::arrowedLine(test, CP, closest_pt1, cv::Scalar(0, 255, 0), 2, 8, 0, 0.1);
  // Red arrow for dir2, from CP to closest_pt2
  cv::arrowedLine(test, CP, closest_pt2, cv::Scalar(0, 0, 255), 2, 8, 0, 0.1);

  // 텍스트 추가 (주석 형식으로 실제 화면에 보일 수 있게)
  cv::putText(test, "dir1 (Green line)", closest_pt1 + cv::Point2f(10, 10), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(0, 255, 0), 1);
  cv::putText(test, "dir2 (Blue line)", closest_pt2 + cv::Point2f(10, -10), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(0, 0, 255), 1);

  // 계산된 각도 차이 표시
  std::string diff_text = "Diff: " + std::to_string((int)angle_diff) + " deg";
  cv::putText(test, diff_text, CP + cv::Point2f(-60, 30), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255), 1);
  cv::putText(test, "Blue Dot (CP)", CP + cv::Point2f(-60, 50), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255),
              1);

  // 시각화 창 표시 및 대기
  cv::imshow("Validation Visualization", test);
  cv::waitKey(0); // 사용자가 키를 누를 때까지 대기
#endif

  // 방향이 30도 이상 벌어져 있음 = 다른 방향이 나옴 = 선들 사이에 잘 있음
  return true;
}
auto ImgPreprocessor::getClosestPointOnLine(const cv::Point2f &line_start, const cv::Point2f &line_end,
                                            const cv::Point2f &P0) -> cv::Point2f {
  float dx = line_end.x - line_start.x;
  float dy = line_end.y - line_start.y;
  if (dx == 0 && dy == 0) {
    // 동일 점인 경우 line_start 반환 (error case)
    return line_start;
  }
  float t = ((P0.x - line_start.x) * dx + (P0.y - line_start.y) * dy) / (dx * dx + dy * dy);
  if (t < 0) {
    return line_start; // 점이 선분의 line_start쪽에 가장 가까움 (error case)
  } else if (t > 1) {
    return line_end; // 점이 선분의 line_end 쪽에 가장 가까움 (error case)
  }
  return cv::Point2f(line_start.x + t * dx, line_start.y + t * dy); // 선분 위의 점 반환
}

auto ImgPreprocessor::isKeyPointNearLine(const cv::KeyPoint &keypoint, const cv::Point2f &lineStart,
                                         const cv::Point2f &lineEnd, double maxDistance) -> bool {
  double dx = lineEnd.x - lineStart.x;
  double dy = lineEnd.y - lineStart.y;

  // 분자 계산
  double numerator = dy * keypoint.pt.x - dx * keypoint.pt.y + lineEnd.x * lineStart.y - lineEnd.y * lineStart.x;
  // 분모의 제곱 계산
  double denominator_sq = dy * dy + dx * dx;

  // sqrt 연산을 피하기 위해 양변을 제곱하여 비교 (거리^2 <= 최대거리^2)
  return (numerator * numerator) <= (maxDistance * maxDistance * denominator_sq);
}

auto ImgPreprocessor::findClosestBaselineDotToNearCP() -> void {

  cv::KeyPoint closestKeyPoint;
  float        minDistance = std::numeric_limits<float>::max();

  if (nth_closest_keypoints_on_baseline.empty()) {
    return;
  }
  for (const auto &keypoint : nth_closest_keypoints_on_baseline[0]) {
    float distance = cv::norm(keypoint.pt - closest_dot_to_cross_point.pt);
    if (distance < minDistance) {
      minDistance     = distance;
      closestKeyPoint = keypoint;
    }
  }
  if (minDistance != std::numeric_limits<float>::max()) {
    closest_baseline_dot = (closestKeyPoint);
  }
}

void ImgPreprocessor::drawClusteredLines() {
  if (src.type() == CV_8UC1) {
    cv::cvtColor(src, src, COLOR_GRAY2BGR);
  }
  for (const auto &ke : clustered_lines) {
    cv::line(src, cv::Point(ke[0].pt1.x, ke[0].pt1.y), cv::Point(ke[0].pt2.x, ke[0].pt2.y), Scalar(0, 0, 255), 1);
  }
#ifdef SHOW_VIDEO
  imshow("drawClusteredLines", this->src);
#endif
}

auto ImgPreprocessor::findwarpPerspectiveCornerPoint() -> bool {
  // 1. 방어적 코드: closestLines가 최소 3개 이상 있어야 안전함
  if (closestLines.size() < 3) {
    LBS_error_code = findwarpPerspectiveCornerPoint_sidelines_error;
    return false;
  }

  std::vector<Line<int>> sideLines = {closestLines[1][0], closestLines[2][0]};

// [디버그용 시각화]
#ifdef SHOW_VIDEO
  cv::Mat test = src.clone();
  for (const auto &line : sideLines) {
    cv::line(test, line.pt1, line.pt2, {0, 255, 0}, 2);
  }
  cv::imshow("test222", test);
#endif
  // 2. 각 라인별로 기준점(closest_baseline_dot)과 가장 가까운 점 찾기
  const std::vector<int> side_line_indices = {1, 2};

  for (int l_idx : side_line_indices) {
    // 인덱스 초과 접근 방지 (안전장치)
    if (l_idx >= closestLines.size() || l_idx >= nth_closest_keypoints_on_baseline.size()) {
      continue;
    }

    const auto &line_group = closestLines[l_idx];
    const auto &keypoints  = this->nth_closest_keypoints_on_baseline[l_idx];

    // 라인이 비어있을 경우의 예외 처리
    if (line_group.empty()) {
      if (!is_crosshair_on_baseline) {
        LBS_error_code = findwarpPerspectiveCornerPoint_fatal_error;
        return false;
      } else {
        // 빈 라인인 경우 기본값 삽입 후 패스
        side_line_info.emplace_back(closest_baseline_dot.pt, Line<int>{}, l_idx);
        continue;
      }
    }

    // [최적화 유지] sqrt 대신 '제곱 거리'를 이용해 기준점과 가장 가까운 점 탐색
    float             min_dist_sq  = std::numeric_limits<float>::max();
    cv::Point2f       min_dist_dot = closest_baseline_dot.pt;
    const cv::Point2f target_pt    = closest_baseline_dot.pt;

    for (const auto &ke : keypoints) {
      float dx      = target_pt.x - ke.pt.x;
      float dy      = target_pt.y - ke.pt.y;
      float dist_sq = (dx * dx) + (dy * dy);

      if (dist_sq < min_dist_sq) {
        min_dist_sq  = dist_sq;
        min_dist_dot = ke.pt;
      }
    }

    // 계산된 가장 가까운 점과 라인 정보 저장
    side_line_info.emplace_back(min_dist_dot, line_group[0], l_idx);
  }

// [디버그용 시각화] 코너 닷 후보군
#ifdef SHOW_VIDEO
  for (const auto &dot : nth_closest_keypoints_on_baseline[std::get<2>(side_line_info[0])]) {
    cv::circle(src, dot.pt, dot.size, {0, 255, 0}, -1);
  }
  for (const auto &dot : nth_closest_keypoints_on_baseline[std::get<2>(side_line_info[1])]) {
    cv::circle(src, dot.pt, dot.size * 1.2, {0, 0, 255}, -1);
  }
  constexpr int CP_size = 6;
  cv::Point     cp(static_cast<int>(img_cross_point.x), static_cast<int>(img_cross_point.y));
  cv::line(src, cv::Point(cp.x - CP_size, cp.y), cv::Point(cp.x + CP_size, cp.y), {233, 233, 1}, 1);
  cv::line(src, cv::Point(cp.x, cp.y - CP_size), cv::Point(cp.x, cp.y + CP_size), {233, 233, 1}, 1);
  cv::imshow("corner dots candidate", src);
  cv::waitKey(0);
#endif //! SHOW_VIDEO

  if (side_line_info.size() < 2)
    return false; // 안전 장치

  // 3. 확장 점(Corner Dot) 찾기 로직 (최대 2회 재시도)
  int                       start_expansion1 = 0, end_expansion1 = 0, start_expansion2 = 0, end_expansion2 = 0;
  std::vector<cv::KeyPoint> dots_to_start1, dots_to_end1, dots_to_start2, dots_to_end2;

  for (int retry = 0; retry < 2; retry++) {
    // 첫 시도 실패 시 (retry == 1) 첫 번째 선분의 방향을 뒤집고 데이터 초기화 후 재시도
    if (retry == 1) {
      if (this->two_Corner_dot_on_side_Line1.size() >= 2 && this->two_Corner_dot_on_side_Line2.size() >= 2) {
        break; // 이미 점을 충분히 찾았다면 루프 탈출
      }

      std::swap(std::get<1>(side_line_info[0]).pt1, std::get<1>(side_line_info[0]).pt2);

      start_expansion1 = end_expansion1 = start_expansion2 = end_expansion2 = 0;
      dots_to_start1.clear();
      dots_to_end1.clear();
      dots_to_start2.clear();
      dots_to_end2.clear();
      this->two_Corner_dot_on_side_Line1.clear();
      this->two_Corner_dot_on_side_Line2.clear();
    }

    // 가독성을 위한 레퍼런스(참조) 변수 선언 (std::get 남발 방지)
    auto &info1_dot  = std::get<0>(side_line_info[0]);
    auto &info1_line = std::get<1>(side_line_info[0]);
    auto &info1_pts  = nth_closest_keypoints_on_baseline[std::get<2>(side_line_info[0])];

    auto &info2_dot  = std::get<0>(side_line_info[1]);
    auto &info2_line = std::get<1>(side_line_info[1]);
    auto &info2_pts  = nth_closest_keypoints_on_baseline[std::get<2>(side_line_info[1])];

    findCornerDotCandidates(info1_line.pt1, info1_line.pt2, info1_dot, info1_pts, start_expansion1, end_expansion1,
                            dots_to_start1, dots_to_end1);

    findCornerDotCandidates(info2_line.pt1, info2_line.pt2, info2_dot, info2_pts, start_expansion2, end_expansion2,
                            dots_to_start2, dots_to_end2);

    // 4. 조건에 따른 점 삽입 (복잡한 조건문을 직관적으로 정리)
    bool        is_start_expansion = (start_expansion1 != 0 || start_expansion2 != 0);
    bool        is_end_expansion   = (end_expansion1 != 0 || end_expansion2 != 0);
    const float pt_size            = 2.0f;
    const int   exp_num            = this->expansion_point_num; // 타이핑 및 가독성 개선

    if (is_start_expansion) {
      int max_exp = std::max(start_expansion1, start_expansion2);

      if (dots_to_start1.size() >= exp_num + max_exp && dots_to_start2.size() >= exp_num + max_exp &&
          dots_to_end1.size() >= exp_num - max_exp && dots_to_end2.size() >= exp_num - max_exp) {

        this->two_Corner_dot_on_side_Line1.push_back(dots_to_start1[max_exp + exp_num - 1]);
        this->two_Corner_dot_on_side_Line2.push_back(dots_to_start2[max_exp + exp_num - 1]);

        if (max_exp == exp_num) {
          this->two_Corner_dot_on_side_Line1.push_back({info1_dot, pt_size});
          this->two_Corner_dot_on_side_Line2.push_back({info2_dot, pt_size});
        } else {
          this->two_Corner_dot_on_side_Line1.push_back(dots_to_end1[exp_num - max_exp - 1]);
          this->two_Corner_dot_on_side_Line2.push_back(dots_to_end2[exp_num - max_exp - 1]);
        }
      }
    } else if (is_end_expansion) {
      int max_exp = std::max(end_expansion1, end_expansion2);

      if (dots_to_end1.size() >= exp_num + max_exp && dots_to_end2.size() >= exp_num + max_exp &&
          dots_to_start1.size() >= exp_num - max_exp && dots_to_start2.size() >= exp_num - max_exp) {

        this->two_Corner_dot_on_side_Line1.push_back(dots_to_end1[max_exp + exp_num - 1]);
        this->two_Corner_dot_on_side_Line2.push_back(dots_to_end2[max_exp + exp_num - 1]);

        if (max_exp == exp_num) {
          this->two_Corner_dot_on_side_Line1.push_back({info1_dot, pt_size});
          this->two_Corner_dot_on_side_Line2.push_back({info2_dot, pt_size});
        } else {
          this->two_Corner_dot_on_side_Line1.push_back(dots_to_start1[exp_num - max_exp - 1]);
          this->two_Corner_dot_on_side_Line2.push_back(dots_to_start2[exp_num - max_exp - 1]);
        }
      }
    } else {
      // 깔끔하게 떨어지는 경우
      if (dots_to_start1.size() >= exp_num && dots_to_end1.size() >= exp_num && dots_to_start2.size() >= exp_num &&
          dots_to_end2.size() >= exp_num) {
        this->two_Corner_dot_on_side_Line1.push_back(dots_to_start1[exp_num - 1]);
        this->two_Corner_dot_on_side_Line1.push_back(dots_to_end1[exp_num - 1]);
        this->two_Corner_dot_on_side_Line2.push_back(dots_to_start2[exp_num - 1]);
        this->two_Corner_dot_on_side_Line2.push_back(dots_to_end2[exp_num - 1]);
      }
    }
  }

  // 최종 유효성 검사
  if (two_Corner_dot_on_side_Line1.size() < 2 || two_Corner_dot_on_side_Line2.size() < 2) {
    LBS_error_code = findwarpPerspectiveCornerPoint_sidelines_error;
    return false;
  }

  return true;
}

auto ImgPreprocessor::findCornerDotCandidates(const cv::Point2f &lineStart, const cv::Point2f &lineEnd,
                                              const cv::Point2f &startDot, const std::vector<cv::KeyPoint> &keypoints,
                                              int &start_expansion, int &end_expansion,
                                              std::vector<cv::KeyPoint> &dots_to_start,
                                              std::vector<cv::KeyPoint> &dots_to_end) -> bool {

  cv::Point2f directionToStart = lineStart - startDot;
  cv::Point2f directionToEnd   = lineEnd - startDot;

  // 확장 기준점에서 선분의 해당 방향으로의 점의 갯수
  std::vector<std::pair<float, cv::Point2f>> points_to_start, points_to_end;

  for (int i = 0; i < keypoints.size(); ++i) {
    // 동일 위치 제거
    if (cv::norm(keypoints[i].pt - startDot) <= std::numeric_limits<float>::epsilon())
      continue;

    // 방향이 맞는지 확인 (dotProduct가 양수인지 -> 양수면은 해당 방향의 정보가 있는 곳으로 pushback 한다.)
    cv::Point2f to_dot               = keypoints[i].pt - startDot;
    float       dot_product_to_start = to_dot.dot(directionToStart);
    float       dot_product_to_end   = to_dot.dot(directionToEnd);
    if (dot_product_to_start > 0) {
      float distance = cv::norm(to_dot);
      points_to_start.emplace_back(distance, cv::Point2f(keypoints[i].pt));
    }
    if (dot_product_to_end > 0) {
      float distance = cv::norm(to_dot);
      points_to_end.emplace_back(distance, cv::Point2f(keypoints[i].pt));
    }
  }
  // dist 오름차순 정렬
  auto sortByDistance = [](const std::pair<float, Point2f> &a, const std::pair<float, Point2f> &b) {
    return a.first < b.first;
  };
  std::sort(points_to_start.begin(), points_to_start.end(), sortByDistance);
  std::sort(points_to_end.begin(), points_to_end.end(), sortByDistance);

  for (const auto &pt : points_to_start) {
    dots_to_start.emplace_back(pt.second, 2);
  }
  for (const auto &pt : points_to_end) {
    dots_to_end.emplace_back(pt.second, 2);
  }

  // 확장 기준점으로부터 선분의 start 방향에 점이 한개도 없다면 end방향 점들의 갯수가 expansion_point_num만큼 더
  // 있어야함.
  if (points_to_start.size() == 0) {
    end_expansion = this->expansion_point_num;
  }
  // expansion_point_num보다 적게 있다면 반대 방향에 적은 숫자만큼 더 있어야함.
  else if (points_to_start.size() < expansion_point_num) {
    end_expansion = expansion_point_num - points_to_start.size();
  }

  if (points_to_end.size() == 0) {
    start_expansion = this->expansion_point_num;
  } else if (points_to_end.size() < expansion_point_num) {
    start_expansion = expansion_point_num - points_to_end.size();
  }

  return true;
}

auto ImgPreprocessor::isRectangleWithMargin(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4, double margin)
    -> bool {
  auto angleDiff = [](double angle1, double angle2) {
    double diff = fabs(angle1 - angle2);
    return std::min(diff, CV_PI - diff);
  };

  // 각도 계산
  double angle1 = atan2(p2.y - p1.y, p2.x - p1.x);
  double angle2 = atan2(p3.y - p2.y, p3.x - p2.x);
  double angle3 = atan2(p4.y - p3.y, p4.x - p3.x);
  double angle4 = atan2(p1.y - p4.y, p1.x - p4.x);

  // 변의 길이 계산
  double dist1 = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
  double dist2 = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2));
  double dist3 = sqrt(pow(p4.x - p3.x, 2) + pow(p4.y - p3.y, 2));
  double dist4 = sqrt(pow(p1.x - p4.x, 2) + pow(p1.y - p4.y, 2));

  // 변 길이와 각도 비교
  bool lengthCheck1 = fabs(dist1 - dist3) <= dist1 * margin;
  bool lengthCheck2 = fabs(dist2 - dist4) <= dist2 * margin;
  bool angleCheck = angleDiff(angle1, angle3) <= CV_PI / 2 * margin && angleDiff(angle2, angle4) <= CV_PI / 2 * margin;

  return lengthCheck1 && lengthCheck2 && angleCheck;
}

void ImgPreprocessor::drawDetectedCornerDots() {
  if (src.type() == CV_8UC1) {
    cv::cvtColor(src, src, COLOR_GRAY2BGR);
  }
  for (auto &ke : two_Corner_dot_on_side_Line1) {
    cv::circle(src, ke.pt, 10, {204, 51, 0}, -1);
    ke.size = 20;
  }
  for (auto &ke : two_Corner_dot_on_side_Line2) {
    cv::circle(src, ke.pt, 10, {0, 153, 255}, -1);
    ke.size = 20;
  }

  // cv::drawKeypoints(src, two_Corner_dot_on_side_Line1, src, cv::Scalar(255, 255, 0),
  // cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); cv::drawKeypoints(src, two_Corner_dot_on_side_Line2, src,
  // cv::Scalar(0, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

auto ImgPreprocessor::arrangeDstQuad(const std::vector<cv::Point2f> &warp_point_candidate) {
  std::vector<cv::Point2f> dst_quad(4);

  // 첫 번째 쌍과 두 번째 쌍의 중점을 계산
  cv::Point2f midpoint1 = (warp_point_candidate[0] + warp_point_candidate[1]) * 0.5f;
  cv::Point2f midpoint2 = (warp_point_candidate[2] + warp_point_candidate[3]) * 0.5f;

  // 중점 x좌표 같은 경우를 처리 /////////////////////////////
  if (std::abs(midpoint1.x - midpoint2.x) < 1) {
    // y 좌표의 평균을 기준으로 왼쪽과 오른쪽을 구분
    float avg_y = (warp_point_candidate[0].y + warp_point_candidate[1].y + warp_point_candidate[2].y +
                   warp_point_candidate[3].y) /
                  4.0f;

    std::vector<cv::Point2f> left_points, right_points;

    for (const auto &point : warp_point_candidate) {
      if (point.y > avg_y) {
        left_points.push_back(point);
      } else {
        right_points.push_back(point);
      }
    }
    // 왼쪽 점들 중에서 위/아래 결정
    dst_quad[0] = left_points[0].x < left_points[1].x ? left_points[0] : left_points[1]; // top-left
    dst_quad[1] = left_points[0].x < left_points[1].x ? left_points[1] : left_points[0]; // bottom-left

    double norm1 = norm(dst_quad[1] - right_points[0]);
    double norm2 = norm(dst_quad[1] - right_points[1]);
    dst_quad[2]  = norm1 < norm2 ? right_points[0] : right_points[1]; // bottom-right
    dst_quad[3]  = norm1 < norm2 ? right_points[1] : right_points[0]; // top-right
  } else if (abs(midpoint1.y - midpoint2.y) < 1) {
    int                             left_y_avg         = 0;
    int                             right_y_avg        = 0;
    bool                            is_first_pair_left = midpoint1.x < midpoint2.x;
    const std::vector<cv::Point2f> &left_pair =
        is_first_pair_left ? std::vector<cv::Point2f>{warp_point_candidate[0], warp_point_candidate[1]}
                           : std::vector<cv::Point2f>{warp_point_candidate[2], warp_point_candidate[3]};
    const std::vector<cv::Point2f> &right_pair =
        is_first_pair_left ? std::vector<cv::Point2f>{warp_point_candidate[2], warp_point_candidate[3]}
                           : std::vector<cv::Point2f>{warp_point_candidate[0], warp_point_candidate[1]};

    for (auto it : left_pair) {
      left_y_avg += it.y;
    }
    for (auto it : right_pair) {
      right_y_avg += it.y;
    }
    left_y_avg /= 2;
    right_y_avg /= 2;
    if (left_y_avg < right_y_avg) {
      dst_quad[0] = left_pair[0].x > left_pair[1].x ? left_pair[0] : left_pair[1]; // top-left
      dst_quad[1] = left_pair[0].x > left_pair[1].x ? left_pair[1] : left_pair[0]; // bottom-left
    } else {
      dst_quad[0] = left_pair[0].y < left_pair[1].y ? left_pair[0] : left_pair[1]; // top-left
      dst_quad[1] = left_pair[0].y < left_pair[1].y ? left_pair[1] : left_pair[0]; // bottom-left
    }
    double norm1 = norm(dst_quad[1] - right_pair[0]);
    double norm2 = norm(dst_quad[1] - right_pair[1]);
    dst_quad[2]  = norm1 < norm2 ? right_pair[0] : right_pair[1]; // bottom-right
    dst_quad[3]  = norm1 < norm2 ? right_pair[1] : right_pair[0]; // top-right
  } else {
    bool                            is_first_pair_left = midpoint1.x < midpoint2.x;
    const std::vector<cv::Point2f> &left_pair =
        is_first_pair_left ? std::vector<cv::Point2f>{warp_point_candidate[0], warp_point_candidate[1]}
                           : std::vector<cv::Point2f>{warp_point_candidate[2], warp_point_candidate[3]};
    const std::vector<cv::Point2f> &right_pair =
        is_first_pair_left ? std::vector<cv::Point2f>{warp_point_candidate[2], warp_point_candidate[3]}
                           : std::vector<cv::Point2f>{warp_point_candidate[0], warp_point_candidate[1]};

    dst_quad[0] = left_pair[0].y < left_pair[1].y ? left_pair[0] : left_pair[1]; // top-left
    dst_quad[1] = left_pair[0].y < left_pair[1].y ? left_pair[1] : left_pair[0]; // bottom-left

    double norm1 = norm(dst_quad[1] - right_pair[0]);
    double norm2 = norm(dst_quad[1] - right_pair[1]);

    dst_quad[2] = norm1 < norm2 ? right_pair[0] : right_pair[1]; // bottom-right
    dst_quad[3] = norm1 < norm2 ? right_pair[1] : right_pair[0]; // top-right
  }

  return dst_quad;
}

auto ImgPreprocessor::adjustCornersToPredefinedArea() -> bool {

  this->dst_quad.resize(4);
  this->src_quad.resize(4);

  vector<cv::Point2f> warp_point_candidate;
  warp_point_candidate.reserve(4);
  warp_point_candidate.push_back(this->two_Corner_dot_on_side_Line1[0].pt);
  warp_point_candidate.push_back(this->two_Corner_dot_on_side_Line1[1].pt);
  warp_point_candidate.push_back(this->two_Corner_dot_on_side_Line2[0].pt);
  warp_point_candidate.push_back(this->two_Corner_dot_on_side_Line2[1].pt);

  dst_quad = arrangeDstQuad(warp_point_candidate);

  // draw warpPerspective order

  for (int i = 0; i < 3; i++) {
    line(src, dst_quad[i], dst_quad[i + 1], Scalar(255, 0, (i == 0 ? 255 : 0)), 4);
  }

  src_quad = {Point2f(0, 0), Point2f(0, ROI_height_size - 1), Point2f(ROI_width_size - 1, ROI_height_size - 1),
              Point2f(ROI_width_size - 1, 0)};

  if (isRectangleWithMargin(dst_quad[0], dst_quad[1], dst_quad[2], dst_quad[3], 0.3)) {
    cv::Mat pers;
    try {
      pers = cv::getPerspectiveTransform(dst_quad, src_quad);

      cv::Point2f img_up_point = img_cross_point + cv::Point2f(0.0f, -10.0f); // 10픽셀 위쪽 점

      // 3. 두 점을 dst_quad 좌표계로 변환합니다.
      // perspectiveTransform은 벡터(std::vector)를 인자로 받습니다.
      std::vector<cv::Point2f> src_pts = {img_cross_point, img_up_point};
      std::vector<cv::Point2f> dst_pts;
      cv::perspectiveTransform(src_pts, dst_pts, pers);

      // 4. dst 공간에서의 변위 벡터를 구합니다.
      cv::Point2f transformed_vec = dst_pts[1] - dst_pts[0];

      // 5. atan2를 이용하여 각도(radian)를 구합니다.
      // 결과값은 -PI ~ PI 범위입니다.
      double angle_rad = std::atan2(transformed_vec.y, transformed_vec.x);

      // 필요하다면 degree로 변환 (0도: 우측, 90도: 하단, -90도: 상단)
      camera_up_angle_ = angle_rad * 180.0 / CV_PI;

    } catch (...) {
      LBS_error_code = adjustCornersToPredefinedArea_Transform_error;
      return false;
    }

    cv::warpPerspective(this->gray, warpPerspectived_image, pers,
                        cv::Size(ROI_width_size, static_cast<int>(ROI_height_size)));

    cv::warpPerspective(this->binarized, warp_source_orient, pers,
                        cv::Size(ROI_width_size, static_cast<int>(ROI_height_size)));

  } else {
    LBS_error_code = adjustCornersToPredefinedArea_shape_error;
    return false;
  }
  return true;
}

auto ImgPreprocessor::isAngleSimilar(double angle1, double angle2, double tolernace) -> bool {
  double differ = std::abs(angle1 - angle2); // 두 앵글값의 차
  if (differ > CV_PI) {
    differ = 2 * CV_PI - differ;
  }
  return differ <= tolernace;
}

auto ImgPreprocessor::preprocessPointData() -> bool {
  Mat                      warpMatrix              = cv::getPerspectiveTransform(this->dst_quad, this->src_quad);
  std::vector<cv::Point2f> transformed_cross_point = {this->img_cross_point};

  // 4개의 코너점을 통해 영역을 형성하고 그 영역안에 dot이 있는지 검사.
  warpPerspectiveArea.reserve(warpPerspectiveArea.size() + dst_quad.size());
  warpPerspectiveArea.insert(warpPerspectiveArea.end(), dst_quad.begin(), dst_quad.end());

  size_t margin = 10;
  int    max_x  = std::max({dst_quad[0].x, dst_quad[1].x, dst_quad[2].x, dst_quad[3].x}) + margin;
  int    min_x  = std::min({dst_quad[0].x, dst_quad[1].x, dst_quad[2].x, dst_quad[3].x}) - margin;
  int    max_y  = std::max({dst_quad[0].y, dst_quad[1].y, dst_quad[2].y, dst_quad[3].y}) + margin;
  int    min_y  = std::min({dst_quad[0].y, dst_quad[1].y, dst_quad[2].y, dst_quad[3].y}) - margin;
  for (const auto &dot : all_detected_dots_) {
    if (min_x <= dot.pt.x && dot.pt.x <= max_x && min_y <= dot.pt.y && dot.pt.y <= max_y) {
      point_to_transform.push_back(dot.pt);
    }
  }

  const int MARGIN_LOSS_LINES       = 2;
  const int DASHLINES_LOSS_DOTS_NUM = ((ROI_height_dots - 1) / 3) * 2;
  auto      required_minimal_dot =
      ((ROI_width_dots - MARGIN_LOSS_LINES) * (ROI_height_dots - MARGIN_LOSS_LINES)) - DASHLINES_LOSS_DOTS_NUM;
  if ((required_minimal_dot > point_to_transform.size())) {
    LBS_error_code = preprocessPointData_lackPts_error;
    return false;
  }
  size_t      i = 0;
  vector<int> side_line_idx;
  for (const auto &info : side_line_info) {
    side_line_idx.push_back(std::get<2>(info));
  }
  for (const auto &lines : nth_closest_keypoints_on_baseline) {
    bool side_check = false;
    for (const auto &idx : side_line_idx) {
      if (idx == i) {
        side_check = true;
        i++;
        break;
      }
    }
    if (!side_check) {
      for (const auto &dot : nth_closest_keypoints_on_baseline[i]) {
        if (isPointInsidePolygon(warpPerspectiveArea, dot.pt)) {
          middle_baseLine_dots_ROI.push_back(dot.pt);
        }
      }
      i++;
    } else {
      continue;
    }
    if (middle_baseLine_dots_ROI.size() == 0) {
      LBS_error_code = preprocessPointData_noMidBaseLine_dots_error;
      return false;
    }

    cv::perspectiveTransform(middle_baseLine_dots_ROI, middle_baseLine_dots_ROI, warpMatrix);
    this->middle_baseLines_dots_in_ROI.push_back(middle_baseLine_dots_ROI);
  }
  try {
    for (const auto &dot : nth_closest_keypoints_on_baseline[0]) {
      if (isPointInsidePolygon(warpPerspectiveArea, dot.pt)) {
        middle_baseLine_dots_ROI.emplace_back(dot.pt);
      }
    }
  } catch (...) {
    LBS_error_code = preprocessPointData_error;
    return false;
  }

  try {
    cv::perspectiveTransform(point_to_transform, transformed_points, warpMatrix);
    cv::perspectiveTransform(transformed_cross_point, transformed_cross_point, warpMatrix);
  } catch (...) {
    LBS_error_code = preprocessPointData_transform_error;
    return false;
  }
  this->transformed_CP = transformed_cross_point[0];

  return true;
}

inline void ImgPreprocessor::drawCPOnImg() {
  constexpr int CP_size = 6;
  cv::line(this->src, cv::Point(static_cast<int>(img_cross_point.x) - CP_size, static_cast<int>(img_cross_point.y)),
           cv::Point(static_cast<int>(img_cross_point.x) + CP_size, static_cast<int>(img_cross_point.y)), {233, 233, 1},
           1);
  cv::line(this->src, cv::Point(static_cast<int>(img_cross_point.x), static_cast<int>(img_cross_point.y) - CP_size),
           cv::Point(static_cast<int>(img_cross_point.x), static_cast<int>(img_cross_point.y) + CP_size), {233, 233, 1},
           1);
}

auto ImgPreprocessor::excludeMarginDots() -> bool {
  int cut_margin_size = static_cast<int>(this->closest_dot_to_cross_point.size * 2);
  transformed_points.erase(std::remove_if(transformed_points.begin(), transformed_points.end(),
                                          [this, cut_margin_size](const Point &pt) {
                                            return pt.x <= cut_margin_size || pt.y <= cut_margin_size ||
                                                   this->ROI_width_size - cut_margin_size <= pt.x ||
                                                   this->ROI_height_size - cut_margin_size <= pt.y;
                                          }),
                           transformed_points.end());

  // mid baseline의 margin영역또한 지움
  for (auto &mid_lines_dots : middle_baseLines_dots_in_ROI) {
    mid_lines_dots.erase(std::remove_if(mid_lines_dots.begin(), mid_lines_dots.end(),
                                        [this, cut_margin_size](const Point &pt) {
                                          return pt.x <= cut_margin_size || pt.y <= cut_margin_size ||
                                                 this->ROI_width_size - cut_margin_size <= pt.x ||
                                                 this->ROI_height_size - cut_margin_size <= pt.y;
                                        }),
                         mid_lines_dots.end());
    if (mid_lines_dots.size() > 10) {
      LBS_error_code = excludeMarginDots_lackMidPts_error;
      return false;
    }
  }
  return true;
}

auto ImgPreprocessor::isPointInsidePolygon(const std::vector<cv::Point> &polygon, const cv::Point &point) -> bool {
  double result = cv::pointPolygonTest(polygon, point, false);
  return result >= 0;
}

void ImgPreprocessor::drawTransformedDots() {
  if (warpPerspectived_image.type() == CV_8UC1) {
    cv::cvtColor(warpPerspectived_image, warpPerspectived_image, COLOR_GRAY2BGR);
  }
  for (const auto &dot : transformed_points) {
    circle(warpPerspectived_image, dot, 2, cv::Scalar(0, 0, 255), 2, -1);
  }

  // Cross Point 표시
  int size = 10;
  cv::line(this->warpPerspectived_image, cv::Point(transformed_CP.x - size, transformed_CP.y),
           cv::Point(transformed_CP.x + size, transformed_CP.y), {233, 233, 1}, 2);
  cv::line(this->warpPerspectived_image, cv::Point(transformed_CP.x, transformed_CP.y - size),
           cv::Point(transformed_CP.x, transformed_CP.y + size), {233, 233, 1}, 2);

  // 가상 그리드라인 그리기
  float line_start = 0;
  float line_step  = this->ROI_width_size / (ROI_width_dots - 1);
  for (int i = 0; i <= (ROI_width_dots - 1); i++) {
    Point2f start = {line_start, 0};
    Point2f end   = {line_start, 1000};
    line(warpPerspectived_image, start, end, Scalar(0, 0, 255), 1);
    line_start += line_step;
  }
  line_start       = 0;
  float line_step2 = this->ROI_height_size / (ROI_height_dots - 1);
  for (int i = 0; i <= (ROI_height_dots - 1); i++) {
    Point2f start = {0, line_start};
    Point2f end   = {1000, line_start};
    line(warpPerspectived_image, start, end, Scalar(0, 0, 255), 1);
    line_start += line_step2;
  }
}

auto ImgPreprocessor::setupPreprocessing(Mat &source) -> bool {
  this->src = source.clone();

  // 그레이스케일로 변환
  if (src.channels() != 1) {
    cv::cvtColor(this->src, this->gray, COLOR_BGR2GRAY);
  }

  // 노이즈 제거를 위해 가우시안 블러 적용 (매우 작은 사진에서는 유의해야 함)
  cv::GaussianBlur(this->gray, this->gray, cv::Size(5, 5), 0);

  return true;
}

auto ImgPreprocessor::mergeCloseLines(const vector<Vec4i> &lines, float maxDistance) -> vector<Vec4i> {
  vector<Vec4i> mergedLines;
  vector<bool>  merged(lines.size(), false); // 선분이 합쳐졌는지 여부를 추적하는 벡터

  for (size_t i = 0; i < lines.size(); ++i) {
    if (merged[i])
      continue; // 이미 합쳐진 선분은 건너뛴다

    Vec4i currentLine = lines[i];
    Point start(currentLine[0], currentLine[1]), end(currentLine[2], currentLine[3]);

    // 현재 선분과 가까운 다른 선분을 찾아 합친다
    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (merged[j])
        continue; // 이미 합쳐진 선분은 건너뛴다

      Vec4i compare_line = lines[j];
      Point start_compare(compare_line[0], compare_line[1]), end_compare(compare_line[2], compare_line[3]);

      // 선분의 시작점 또는 끝점이 가까운 경우 선분을 합친다
      if (norm(start - start_compare) < maxDistance || norm(end - end_compare) < maxDistance ||
          norm(start - end_compare) < maxDistance || norm(end - start_compare) < maxDistance) {
        // 선분을 합치기 위해 시작점과 끝점을 갱신한다
        start     = Point(min(start.x, start_compare.x), min(start.y, start_compare.y));
        end       = Point(max(end.x, end_compare.x), max(end.y, end_compare.y));
        merged[j] = true; // 선분을 합쳤음을 표시
      }
    }
    mergedLines.emplace_back(start.x, start.y, end.x, end.y);
  }

  return mergedLines;
}

void ImgPreprocessor::normalizeLinePoints(vector<Vec4i> &lines) {
  for (auto &line : lines) {
    Point start(line[0], line[1]);
    Point end(line[2], line[3]);

    // 시작점이 끝점보다 오른쪽에 있거나, 더 아래에 있는 경우, 시작점과 끝점을 교환
    if (start.x > end.x || start.y > end.y) {
      swap(line[0], line[2]);
      swap(line[1], line[3]);
    }
  }
}

void ImgPreprocessor::setHoughPAndFindOrientLines(cv::Mat &img_ROI, std::vector<Vec4i> &lines) {
  // HoughLinesP를 위한 세팅값
  int threshold_value = 60;
  int minLineLength   = 30;
  int maxLineGap      = 10;

  HoughLinesP(img_ROI, lines, 1, CV_PI / 180, threshold_value, minLineLength, maxLineGap);
}

auto ImgPreprocessor::checkAndCorrectOrientation() -> bool {
  bool is_flipped = false;

  // 중앙 Triple-Order Line 체크를 위한 영역 설정 (이미지 뒤집힘 체크)
  int         size_roi = 50;                                                       // 임의 설정
  cv::Point2i start_pt = {static_cast<int>(this->warp_source_orient.cols / 2), 0}; // ROI 상단 중앙 좌표
  cv::Rect    roi_area = {start_pt.x - size_roi, start_pt.y, size_roi * 2,
                          static_cast<int>(this->warp_source_orient.rows)};
  Mat         img_ROI  = this->warp_source_orient(roi_area).clone();

  if (img_ROI.channels() != 1) {
    cvtColor(img_ROI, img_ROI, COLOR_BGR2GRAY);
  }

  // 선 소실점 복원에 대한 모폴로지 연산
  Mat kernel     = getStructuringElement(MORPH_RECT, Size(2, 4));
  int close_iter = 1;
  cv::morphologyEx(img_ROI, img_ROI, cv::MORPH_CLOSE, kernel, {}, close_iter);
  cv::threshold(img_ROI, img_ROI, 0, 255, THRESH_BINARY | THRESH_OTSU);

  vector<Vec4i> lines;
  setHoughPAndFindOrientLines(img_ROI, lines);
  normalizeLinePoints(lines);

  int           line_merge_tolerance = 30;
  vector<Vec4i> merged_lines         = mergeCloseLines(lines, line_merge_tolerance);
  // double overlappedLineCut = 10; // test 겹친 선 제거 위한 범위 변수

  // 선 길이,위치 정보를 저장
  vector<pair<float, Point>> linesInfo;
  for (auto l : merged_lines) {
    float length = sqrt(pow(l[2] - l[0], 2) + pow(l[3] - l[1], 2));
    Point line_center_location((l[2] + l[0]) / 2, (l[3] + l[1]) / 2);
    linesInfo.emplace_back(length, line_center_location);
  }

  // 선의 위치(높이)에 따라 정렬
  sort(linesInfo.begin(), linesInfo.end(),
       [](const pair<float, Point> &a, const pair<float, Point> &b) { return a.second.y < b.second.y; });

  if (linesInfo.size() != 0) {
    for (int i = 0; i < linesInfo.size() - 1; i++) {
      double                     temp;
      vector<pair<float, Point>> savedLines = linesInfo;

      // 겹치는 라인 제거
      try {
        if (abs(linesInfo[i + 1].second.y - linesInfo[i].second.y) < 5) {
          temp = linesInfo[i + 1].first >= linesInfo[i].first ? i : i + 1;
          linesInfo.erase(linesInfo.begin() + temp);
        }
      } catch (...) { return false; }
    }
  } else {
    return false;
  }
  // 선 그리기

  // 라인의 갯수가 타겟 범위에 들어오지 않을 경우 false  warpPerspective Size가 13 -> 라인이 4개가 들어옴.
  // int linesInfoSize = ROI_height_dots=
  if (!(linesInfo.size() == 4)) {
    LBS_error_code = orienting_LineNums_error;
#ifdef SHOW_VIDEO
    cout << "orienting_line_num_error, line size : " << linesInfo.size() << endl; // 테스트 코드

    cvtColor(img_ROI, img_ROI, COLOR_GRAY2BGR);
    for (auto l : merged_lines) {
      line(img_ROI, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
    }
    imshow("orienting_line_num_error - detected lines", img_ROI);
    waitKey(1);
#endif //! SHOW_VIDEO
    return false;
  }

  // validate dash line
  float       max_ = max({linesInfo[0].first, linesInfo[1].first, linesInfo[2].first}); // 각 위치 길이 저장..
  const float line_diff_max_tolerence = 1.8;
  if (max_ >= linesInfo[0].first * line_diff_max_tolerence || max_ >= linesInfo[1].first * line_diff_max_tolerence ||
      max_ >= linesInfo[2].first * line_diff_max_tolerence) {
    // test

#ifdef SHOW_VIDEO
    imshow("line_diff_error -> max is so big", img_ROI);
    // waitKey(1);
    cout << "orient Line error(max가 나머지것에 비해 1.8배가 넘어감)" << endl;
    cout << "max : " << max_ << endl;
    cout << "1 : " << linesInfo[0].first << "," << linesInfo[1].first << "," << linesInfo[2].first << endl;
#endif //! SHOW_VIDEO

    LBS_error_code = orienting_line_max_too_big_error;
    return false;
  }
  float temp_a       = std::abs(linesInfo[0].first - linesInfo[1].first);
  float temp_b       = std::abs(linesInfo[1].first - linesInfo[2].first);
  float temp_c       = std::abs(linesInfo[2].first - linesInfo[0].first);
  float len_diff_min = 6;
  if (temp_a < len_diff_min || temp_b < len_diff_min || temp_c < len_diff_min) {

#ifdef SHOW_VIDEO
    cvtColor(img_ROI, img_ROI, COLOR_GRAY2BGR);
    for (auto l : merged_lines) {
      line(img_ROI, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
    }

    cout << "diff_min_error" << endl;
    cout << "Line Top " << linesInfo[0].first << ", Line mid:" << linesInfo[1].first
         << ", Line bot  : " << linesInfo[2].first << endl;
    imshow("diff_min_error", img_ROI);
    // waitKey(1);
#endif //! SHOW_VIDEO

    LBS_error_code = orienting_cantKnowLineDiff_error;
    return false;
  }

  // check orient
  if (linesInfo.size() == 4) {
    int top = static_cast<int>(linesInfo[0].first);
    int mid = static_cast<int>(linesInfo[1].first);
    int bot = static_cast<int>(linesInfo[2].first);

    vector<int> length_order = {top, mid, bot};
    std::sort(length_order.begin(), length_order.end(), [](int a, int b) { return a > b; });

    int A = length_order[0];
    int B = length_order[1];

    if (A == top) {
      if (B != mid) {
        is_flipped = true;
      }
    } else if (A == mid) {
      if (B != bot) {
        is_flipped = true;
      }
    } else if (A == bot) {
      if (B != top) {
        is_flipped = true;
      }
    }
  } else {
    LBS_error_code = orienting_lackLineNums_error;
    return false;
  }

  // transfomed_points _filp
  if (is_flipped) {
    for (auto &point : this->transformed_points) {
      point.x = this->ROI_width_size - point.x;
      point.y = this->ROI_height_size - point.y;
    }
    this->transformed_CP.x = this->ROI_width_size - transformed_CP.x;
    this->transformed_CP.y = this->ROI_height_size - transformed_CP.y;

    // 뒤집힘 여부에 맞춰 angle 보정 (180도 회전)
    this->camera_up_angle_ = std::abs(this->camera_up_angle_ + 180);
  }

#ifdef SHOW_VIDEO
  imshow("orientation_check", img_ROI);
  std::cout << "flip : " << is_flipped << std::endl;
  waitKey(1);
#endif

  return true;
}

// bool ImgPreprocessor::verifyDotCount() {
//   // 필요한 ROI 내의 전체 점의 수를 계산
//   // subRows, subCols 따라 ROI 내 점의 갯수가 모두 필요하지 않으므로 해당 부분 확인해 느슨한 기준 가능
//   int total_dots = (this->ROI_width_dots) * (this->ROI_height_dots);
//   // 대시라인으로 인해 사라지는 점의 수를 계산
//   int dash_loss = (this->required_baseline_cnt) * 8;

//   int nums_of_required_pts = total_dots - dash_loss;

//   if (all_detected_dots_.size() < nums_of_required_pts) {
//     LBS_error_code = SBD_error;
//     return false;
//   }
//   return true;
// }#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>

/**
 * @brief 이미지에서 원형에 가까운 '점'만 추출하는 함수
 * * @param src 입력 이미지 (cv::Mat)
 * @param minArea 점으로 인정할 최소 면적 (노이즈 제거용, 기본값 5.0)
 * @param maxAspectRatio 점으로 인정할 최대 종횡비 (1에 가까울수록 완벽한 원, 기본값 1.5)
 * @return cv::Mat 점만 남은 결과 이미지
 */
cv::Mat ImgPreprocessor::extractDots(const cv::Mat &src, double minArea, double maxAspectRatio) {
  // 1. 입력 이미지가 3채널(컬러)인 경우 1채널(그레이스케일)로 변환
  cv::Mat gray;
  if (src.channels() == 3) {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = src.clone();
  }

  // 2. 이진화 (배경은 검게, 객체는 하얗게)
  cv::Mat binary = binarizeImage(src);

  // imshow("inbinary", binary);
  // waitKey(0);

  // 3. 외곽선(Contour) 찾기
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 4. 점만 그릴 빈 캔버스 (검은 바탕) 생성
  cv::Mat result = cv::Mat::zeros(gray.size(), CV_8UC1);

  // 5. 조건에 맞는 외곽선 필터링 및 그리기
  for (size_t i = 0; i < contours.size(); i++) {
    double          area    = cv::contourArea(contours[i]);
    cv::RotatedRect rotRect = cv::minAreaRect(contours[i]);

    double width  = rotRect.size.width;
    double height = rotRect.size.height;

    // 크기가 0인 예외 상황 방지
    if (width == 0 || height == 0)
      continue;

    // 종횡비 계산 (긴 변 / 짧은 변)
    double aspectRatio = std::max(width, height) / std::min(width, height);

    // 노이즈(면적)와 선(종횡비)을 제외하고 점만 통과
    if (area > minArea && aspectRatio < maxAspectRatio) {
      cv::drawContours(result, contours, (int)i, cv::Scalar(255), cv::FILLED);
    }
  }

  // 6. 원본처럼 흰 배경에 검은 점으로 보이도록 반전
  cv::Mat finalResult;
  cv::bitwise_not(result, finalResult);

  return finalResult;
}
Mat normalizeIllumination(Mat src) {
  Mat gray;
  if (src.channels() == 3)
    cvtColor(src, gray, COLOR_BGR2GRAY);
  else
    gray = src.clone();

  // 1. 조명 성분 추출 (매우 큰 커널로 블러 처리)
  // 커널 사이즈는 점(Dot)의 크기보다 훨씬 커야 합니다 (예: 101, 151 등)
  Mat illuminationMap;
  int kernelSize = 151; // 1
  GaussianBlur(gray, illuminationMap, Size(kernelSize, kernelSize), 0);

  // 2. 배경 나누기 (원본 / 조명 맵)
  // 0으로 나누는 것을 방지하기 위해 정밀도를 float로 변환하여 계산 후 다시 8비트로 복구
  Mat result;
  gray.convertTo(gray, CV_32F);
  illuminationMap.convertTo(illuminationMap, CV_32F);

  divide(gray, illuminationMap, result, 255.0); // 255를 곱해 밝기 유지
  result.convertTo(result, CV_8U);

  return result;
}
std::string ImgPreprocessor::getErrorMessage() {
  switch (this->LBS_error_code) {
  case SBD_error:
    return "[Error 1] SBD_error: Simple Blob Detector initialization or detection failed.";

  case findBaselines_error:
    return "[Error 2] findBaselines_error: Failed to extract structural baselines from the image.";

  case findValidLines_error:
    return "[Error 3] findValidLines_error: Could not find valid lines satisfying the baseline criteria.";

  case validateClosestBaseLines_error:
    return "[Error 4] validateClosestBaseLines_error: Verification of the closest baselines failed.";

  case findwarpPerspectiveCornerPoint_fatal_error:
    return "[Fatal Error 5] findwarpPerspectiveCornerPoint_fatal_error: Critical failure while detecting warp "
           "perspective corner points.";

  case findwarpPerspectiveCornerPoint_sidelines_error:
    return "[Error 6] findwarpPerspectiveCornerPoint_sidelines_error: Failed to detect side lines during corner point "
           "estimation.";

  case adjustCornersToPredefinedArea_Transform_error:
    return "[Error 7] adjustCornersToPredefinedArea_Transform_error: Perspective transformation matrix calculation "
           "failed during corner adjustment.";

  case adjustCornersToPredefinedArea_shape_error:
    return "[Error 8] adjustCornersToPredefinedArea_shape_error: Invalid shape or layout detected when adjusting "
           "corners to the predefined area.";

  case preprocessPointData_lackPts_error:
    return "[Error 9] preprocessPointData_lackPts_error: Insufficient number of total points for preprocessing.";

  case preprocessPointData_noMidBaseLine_dots_error:
    return "[Error 10] preprocessPointData_noMidBaseLine_dots_error: Missing dots on the middle baseline during "
           "preprocessing.";

  case preprocessPointData_error:
    return "[Error 11] preprocessPointData_error: General failure during point data preprocessing.";

  case preprocessPointData_transform_error:
    return "[Error 12] preprocessPointData_transform_error: Coordinate transformation failed during preprocessing.";

  case excludeMarginDots_lackMidPts_error:
    return "[Error 13] excludeMarginDots_lackMidPts_error: Cannot exclude margin dots due to insufficient middle "
           "points.";

  case orienting_LineNums_error:
    return "[Error 14] orienting_LineNums_error: Mismatch or error in the number of lines during orientation "
           "correction.";

  case orienting_line_max_too_big_error:
    return "[Error 15] orienting_line_max_too_big_error: Maximum line value exceeds the allowed threshold.";

  case orienting_cantKnowLineDiff_error:
    return "[Error 16] orienting_cantKnowLineDiff_error: Unable to determine the line difference for orientation.";

  case orienting_lackLineNums_error:
    return "[Error 17] orienting_lackLineNums_error: Insufficient number of lines to determine correct orientation.";

  default:
    return "[Unknown Error] Invalid error code provided.";
  }
}

cv::Mat applyGrayMaskOverlay(const cv::Mat &src, const cv::Mat &mask, int intensity, double alpha = 0.5) {
  cv::Mat dst = src.clone();

  // 1. 마스크 반전 (0 -> 255, 255 -> 0)
  cv::Mat invMask;
  cv::bitwise_not(mask, invMask);

  // 2. 덮어씌울 색상 레이어 생성 및 합성
  cv::Mat grayLayer = cv::Mat::ones(src.size(), src.type()) * intensity;
  cv::Mat blended;
  cv::addWeighted(src, 1.0 - alpha, grayLayer, alpha, 0, blended);

  // 3. 반전된 마스크를 이용하여 0이었던 영역에만 blended 복사
  blended.copyTo(dst, invMask);

  return dst;
}

void ImgPreprocessor::addDotsOnLine() {
  if (all_detected_dots_.empty() || closestLines.empty())
    return;

  // 1. 모든 루프에서 변하지 않는 max_dist 제곱 값을 미리 계산
  double effective_max_dist = all_detected_dots_[0].size * 1.15;
  double max_dist_sq        = effective_max_dist * effective_max_dist;

  for (auto &line : closestLines) {
    auto &l = line[0]; // 가독성을 위해 참조 변수 지정

    // 선분 고유의 값들을 내부 루프 진입 전에 딱 한 번만 계산
    double dx = static_cast<double>(l.pt2.x - l.pt1.x); // pt2로 수정
    double dy = static_cast<double>(l.pt2.y - l.pt1.y);

    double denominator_sq    = dy * dy + dx * dx;
    double max_dist_sq_denom = max_dist_sq * denominator_sq;
    double line_const        = static_cast<double>(l.pt2.x) * l.pt1.y - static_cast<double>(l.pt2.y) * l.pt1.x;

    // 3. 내부 루프에서는 오직 '점'과 관련된 최소한의 연산만 수행
    for (const auto &dot : all_detected_dots_) {
      double numerator = dy * dot.pt.x - dx * dot.pt.y + line_const;

      if ((numerator * numerator) <= max_dist_sq_denom) {
        l.dots.push_back(dot); // push_back 오버헤드가 우려된다면 아래 병렬화 참고
      }
    }
  }
}
auto ImgPreprocessor::run(cv::Mat source) -> bool {
  // 이미지 없음
  if (source.empty()) {
    return false;
  }

  // 1. 이미지 전처리
  if (!this->setupPreprocessing(source)) {
    return false;
  };

  // 1-1) 조명 보정 (normalize illumination)
  gray = normalizeIllumination(this->gray);
#ifdef SHOW_VIDEO
  imshow("nomalized_light", gray);
#endif
  // 1-2) 대비 보정 (normalize contrast)
  cv::normalize(gray, gray, 0, 150, cv::NORM_MINMAX);
#ifdef SHOW_VIDEO
  imshow("nomalized_contrast_light", this->gray);
  cv::waitKey(0);
#endif
  // 1-3) findcontours 및 필터링으로 선과 잡음 제거
  double  maxAspectRatio = 1.7;
  cv::Mat dots_extracted = extractDots(this->gray, 7.0, maxAspectRatio);
  dots_extracted         = ~dots_extracted;

  // 1-4) 이진화
  binarized = binarizeImage(gray);

#ifdef SHOW_VIDEO
  imshow("dots_extracted", dots_extracted);
  imshow("binarized", binarized);
  cv::waitKey(0);
#endif

  // 2. Dot 검출기 생성 및 점 검출
  DotDetector dotDetector;
  all_detected_dots_ = dotDetector.detectBylable(dots_extracted);

  // 선그리기
  drawDetectedDots(this->src, all_detected_dots_);

  // 찾은 점이 필요 갯수 이하 시 예외 처리
  constexpr int required_min_dots = 49; // 7X7 기준
  if (all_detected_dots_.size() < required_min_dots) {
    LBS_error_code = SBD_error;
    return false;
  }

  // 3. 선 검출에 방해가 되는 점 제거
  eraseDetectedDots(binarized, all_detected_dots_);

  // 엣지 검출
  cv::Canny(binarized, edge, 0, 0);

  // 선 검출
  if (!lineProcessor_.detectBaseLines(edge)) {
    return false;
  }

  clustered_lines = lineProcessor_.getClusteredLines();

  // 검출된 선분 그리기
  this->drawClusteredLines();

  // 이미지 중앙에서 가장 가까운 점을 찾음
  if (findClosestDotToCP()) {
    cv::circle(src, closest_dot_to_cross_point.pt, 5, cv::Scalar(255, 20, 5), -1);
  }

  ///////////////////////////////////////////////
  // 선 위에 점이 기대보다 적은 라인 필터링

  addDotsOnLine();

  this->findValidLinesByDotsLocation();

  // 이미지 중앙에서 가장 가까운점을 기준으로 선분을 정렬
  this->orderBaseLinesByClosestDot();

#ifdef SHOW_VIDEO
  cv::Mat    temp_src = this->src.clone();
  cv::Scalar color(255, 0, 0);  // 제일가까운 B
  cv::Scalar color2(0, 255, 0); // 두번째로 가까운 G
  cv::Scalar color3(0, 0, 255); // 세번째로 가까운 R
  for (int i = 0; i < closestLines.size(); i++) {
    for (const auto &line : closestLines[i]) {
      if (i == 0) {
        cv::line(temp_src, line.pt1, line.pt2, color, 2);
      } else if (i == 1) {
        cv::line(temp_src, line.pt1, line.pt2, color2, 2);
      } else if (i == 2) {
        cv::line(temp_src, line.pt1, line.pt2, color3, 2);
      }
    }
  }
  imshow("orderBaseLinesByClosestDot", temp_src);
  waitKey(0);
#endif

  if (!this->findValidLinesByDotsNum()) {
    LBS_error_code = findValidLines_error;
    return false;
  }

#ifdef SHOW_VIDEO
  {
    // cv::Mat    after_findValidLinesByDotsNum = this->src.clone();
    // cv::Scalar color(255, 0, 0);  // 제일가까운 B
    // cv::Scalar color2(0, 255, 0); // 두번째로 가까운 G
    // cv::Scalar color3(0, 0, 255); // 세번째로 가까운 R
    // for (int i = 0; i < closestLines.size(); i++) {
    //   for (const auto &line : closestLines[i]) {
    //     if (i == 0) {
    //       cv::line(after_findValidLinesByDotsNum, line.pt1, line.pt2, color, 2);
    //     } else if (i == 1) {
    //       cv::line(after_findValidLinesByDotsNum, line.pt1, line.pt2, color2, 2);
    //     } else if (i == 2) {
    //       cv::line(after_findValidLinesByDotsNum, line.pt1, line.pt2, color3, 2);
    //     }
    //   }
    // }
    // imshow("after_findValidLinesByDotsNum", after_findValidLinesByDotsNum);
    // cv::waitKey(0);
  }
#endif

  if (!this->validateClosestBaseLinesByDirection()) {
    this->LBS_error_code = validateClosestBaseLines_error;
    return false;
  }

  this->findClosestBaselineDotToNearCP(); // okay

#ifdef SHOW_VIDEO
  cv::Mat baseline_dot_closest = src.clone();
  cv::circle(baseline_dot_closest, closest_baseline_dot.pt, 6, {255, 22, 100}, 4);
  imshow("baseline_closest_dot", baseline_dot_closest);
  cv::waitKey(0);
#endif

  if (!this->findwarpPerspectiveCornerPoint()) {
    return false;
  }
#ifdef SHOW_VIDEO
  cv::Mat after_findwarpPerspectiveCornerPoint = src.clone();
  for (const auto &line : side_line_info) {
    cv::line(after_findwarpPerspectiveCornerPoint, get<1>(line).pt1, get<1>(line).pt2, {255, 0, 0}, 2);
  }
  cv::imshow("after_findwarpPerspectiveCornerPoint", after_findwarpPerspectiveCornerPoint);
  cv::waitKey(0);
#endif

  // 검출된 코너점 그리기
  this->drawDetectedCornerDots();

  if (!this->adjustCornersToPredefinedArea()) {
    return false;
  }

  if (!this->preprocessPointData()) {
    return false;
  }

  if (!this->excludeMarginDots()) {
    return false;
  }

  this->drawTransformedDots();

  if (!this->checkAndCorrectOrientation()) {
    return false;
  }

#ifdef SHOW_VIDEO
  imshow("warpTest", this->warpPerspectived_image);
#endif
  return true;
}
} // namespace LCODE
