import argparse
import os

import cv2
import cv2.aruco as aruco
import numpy as np


DEFAULT_VIDEO_PATH = r"D:\THESIS\walk_test\walk_3kg.MOV"
ARUCO_DICT = aruco.DICT_6X6_250
TRACKED_TAG_IDS = set(range(12))
RUNWAY_LEFT_IDS = [4, 6, 8, 10]
RUNWAY_RIGHT_IDS = [5, 7, 9, 11]
RUNWAY_ROW_PAIRS = [(4, 5), (6, 7), (8, 9), (10, 11)]
RUNWAY_IDS = set(RUNWAY_LEFT_IDS + RUNWAY_RIGHT_IDS)
RUNWAY_ROW_INDEX = {4: 0, 5: 0, 6: 1, 7: 1, 8: 2, 9: 2, 10: 3, 11: 3}
CALIBRATION_FRAME_COUNT = 100
DUPLICATE_ID6_MODE_CHOICES = ("right-as-7", "auto", "off")
MARKER_COLOR = (0, 255, 255)
TEXT_COLOR = (255, 255, 255)
CENTER_COLOR = (0, 200, 0)
GRID_COLOR = (255, 80, 80)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect and annotate AR tag positions for IDs 0-11 in a video."
    )
    parser.add_argument("--video", default=DEFAULT_VIDEO_PATH, help="Path to input video")
    parser.add_argument(
        "--frame-step",
        type=int,
        default=2,
        help="Process every Nth frame to reduce load",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Disable the real-time preview window",
    )
    parser.add_argument(
        "--max-width",
        type=int,
        default=960,
        help="Max display/export width for landscape frames",
    )
    parser.add_argument(
        "--max-height",
        type=int,
        default=540,
        help="Max display/export height for landscape frames",
    )
    parser.add_argument(
        "--duplicate-id6-mode",
        choices=DUPLICATE_ID6_MODE_CHOICES,
        default="right-as-7",
        help="How to handle two detected ID 6 tags when ID 7 is missing",
    )
    return parser.parse_args()


def resize_to_fit(frame, max_width, max_height):
    height, width = frame.shape[:2]
    scale = min(max_width / width, max_height / height, 1.0)
    if scale == 1.0:
        return frame

    new_width = int(width * scale)
    new_height = int(height * scale)
    return cv2.resize(frame, (new_width, new_height))


def marker_center(marker_corners):
    points = marker_corners[0]
    return (float(np.mean(points[:, 0])), float(np.mean(points[:, 1])))


def build_tracked_detections(corners, ids):
    if ids is None:
        return []

    tracked_detections = []
    for index, marker_id in enumerate(ids.flatten()):
        marker_id = int(marker_id)
        if marker_id not in TRACKED_TAG_IDS:
            continue

        tracked_detections.append((marker_id, corners[index]))

    return tracked_detections


def average_x_for_ids(detections, candidate_ids):
    x_values = [marker_center(marker_corners)[0] for marker_id, marker_corners in detections if marker_id in candidate_ids]
    if not x_values:
        return None

    return float(np.mean(x_values))


def remap_duplicate_runway_ids(detections, mode):
    if mode == "off":
        return detections, set(), False

    duplicate_sixes = [item for item in detections if item[0] == 6]
    if len(duplicate_sixes) < 2:
        return detections, set(), False

    if any(marker_id == 7 for marker_id, _ in detections):
        return detections, set(), False

    sorted_sixes = sorted(duplicate_sixes, key=lambda item: marker_center(item[1])[0])

    if mode == "auto":
        left_reference_x = average_x_for_ids(detections, {4, 8, 10})
        right_reference_x = average_x_for_ids(detections, {5, 9, 11})
        if left_reference_x is not None and right_reference_x is not None:
            first_center_x = marker_center(sorted_sixes[0][1])[0]
            second_center_x = marker_center(sorted_sixes[1][1])[0]
            keep_default_cost = abs(first_center_x - left_reference_x) + abs(second_center_x - right_reference_x)
            swap_cost = abs(first_center_x - right_reference_x) + abs(second_center_x - left_reference_x)
            if swap_cost < keep_default_cost:
                sorted_sixes = list(reversed(sorted_sixes))

    remapped_lookup = {
        id(sorted_sixes[0][1]): 6,
        id(sorted_sixes[1][1]): 7,
    }

    resolved_detections = []
    synthetic_ids = set()
    for marker_id, marker_corners in detections:
        if marker_id == 6 and id(marker_corners) in remapped_lookup:
            resolved_id = remapped_lookup[id(marker_corners)]
            resolved_detections.append((resolved_id, marker_corners))
            if resolved_id != 6:
                synthetic_ids.add(resolved_id)
            continue

        resolved_detections.append((marker_id, marker_corners))

    return resolved_detections, synthetic_ids, True


def build_marker_positions(detections):
    return {
        marker_id: marker_center(marker_corners)
        for marker_id, marker_corners in detections
    }


def runway_positions_from_all(positions):
    return {marker_id: positions[marker_id] for marker_id in RUNWAY_IDS if marker_id in positions}


def update_average_accumulators(positions, point_sums, point_counts):
    for marker_id, point in positions.items():
        sum_x, sum_y = point_sums.get(marker_id, (0.0, 0.0))
        point_sums[marker_id] = (sum_x + point[0], sum_y + point[1])
        point_counts[marker_id] = point_counts.get(marker_id, 0) + 1


def build_average_points(point_sums, point_counts):
    averaged_points = {}
    for marker_id, (sum_x, sum_y) in point_sums.items():
        count = point_counts.get(marker_id, 0)
        if count <= 0:
            continue

        averaged_points[marker_id] = (sum_x / count, sum_y / count)
    return averaged_points


def fit_axis_from_rows(points_by_id):
    if len(points_by_id) < 2:
        return None, None

    row_values = np.array([RUNWAY_ROW_INDEX[marker_id] for marker_id in points_by_id], dtype=np.float64)
    x_values = np.array([points_by_id[marker_id][0] for marker_id in points_by_id], dtype=np.float64)
    y_values = np.array([points_by_id[marker_id][1] for marker_id in points_by_id], dtype=np.float64)
    return np.polyfit(row_values, x_values, 1), np.polyfit(row_values, y_values, 1)


def estimate_missing_side_points(side_ids, working_points):
    side_points = {marker_id: working_points[marker_id] for marker_id in side_ids if marker_id in working_points}
    x_model, y_model = fit_axis_from_rows(side_points)
    if x_model is None or y_model is None:
        return

    for marker_id in side_ids:
        if marker_id in working_points:
            continue

        row_value = RUNWAY_ROW_INDEX[marker_id]
        working_points[marker_id] = (
            float(np.polyval(x_model, row_value)),
            float(np.polyval(y_model, row_value)),
        )


def average_lateral_vector(working_points):
    vectors = []
    for left_id, right_id in RUNWAY_ROW_PAIRS:
        if left_id in working_points and right_id in working_points:
            left_point = working_points[left_id]
            right_point = working_points[right_id]
            vectors.append((right_point[0] - left_point[0], right_point[1] - left_point[1]))

    if not vectors:
        return None

    mean_x = float(np.mean([vector[0] for vector in vectors]))
    mean_y = float(np.mean([vector[1] for vector in vectors]))
    return (mean_x, mean_y)


def fill_missing_row_partners(working_points):
    lateral_vector = average_lateral_vector(working_points)
    if lateral_vector is None:
        return

    vector_x, vector_y = lateral_vector
    for left_id, right_id in RUNWAY_ROW_PAIRS:
        has_left = left_id in working_points
        has_right = right_id in working_points

        if has_left and not has_right:
            left_point = working_points[left_id]
            working_points[right_id] = (left_point[0] + vector_x, left_point[1] + vector_y)
        elif has_right and not has_left:
            right_point = working_points[right_id]
            working_points[left_id] = (right_point[0] - vector_x, right_point[1] - vector_y)


def estimate_points_for_drawing(reference_points):
    draw_points = dict(reference_points)
    for _ in range(2):
        estimate_missing_side_points(RUNWAY_LEFT_IDS, draw_points)
        estimate_missing_side_points(RUNWAY_RIGHT_IDS, draw_points)
        fill_missing_row_partners(draw_points)
    return draw_points


def draw_runway_grid(frame, runway_points, is_fixed):
    if not runway_points:
        return

    thickness = 3 if is_fixed else 2
    for ordered_ids in (RUNWAY_LEFT_IDS, RUNWAY_RIGHT_IDS):
        available_ids = [marker_id for marker_id in ordered_ids if marker_id in runway_points]
        for index in range(len(available_ids) - 1):
            start = tuple(int(round(value)) for value in runway_points[available_ids[index]])
            end = tuple(int(round(value)) for value in runway_points[available_ids[index + 1]])
            cv2.line(frame, start, end, GRID_COLOR, thickness)

    for left_id, right_id in RUNWAY_ROW_PAIRS:
        if left_id not in runway_points or right_id not in runway_points:
            continue

        start = tuple(int(round(value)) for value in runway_points[left_id])
        end = tuple(int(round(value)) for value in runway_points[right_id])
        cv2.line(frame, start, end, GRID_COLOR, thickness)


def draw_marker_positions(frame, positions, synthetic_ids):
    for marker_id, point in sorted(positions.items()):
        center = (int(round(point[0])), int(round(point[1])))
        cv2.circle(frame, center, 5, CENTER_COLOR, -1)
        label_suffix = "~" if marker_id in synthetic_ids else ""
        cv2.putText(
            frame,
            f"ID:{marker_id}{label_suffix} ({center[0]}, {center[1]})",
            (center[0] + 8, center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            MARKER_COLOR,
            2,
        )


def format_status_lines(positions, processed_frames, fixed_runway_points, duplicate_fix_applied, duplicate_id6_mode):
    visible_ids = sorted(positions.keys())
    missing_ids = sorted(TRACKED_TAG_IDS.difference(positions.keys()))
    status_lines = [
        f"Tracked IDs: {visible_ids}",
        f"Missing IDs: {missing_ids}",
        f"Detected count: {len(visible_ids)}/{len(TRACKED_TAG_IDS)}",
        f"Duplicate ID 6 mode: {duplicate_id6_mode}",
    ]
    if fixed_runway_points is None:
        status_lines.append(f"Runway calibration: {processed_frames}/{CALIBRATION_FRAME_COUNT} frames")
    else:
        status_lines.append(f"Runway calibration locked at {CALIBRATION_FRAME_COUNT} frames")

    if duplicate_fix_applied:
        status_lines.append("Applied duplicate ID 6 remap to synthetic ID 7")

    return status_lines


def build_draw_marker_inputs(detections):
    if not detections:
        return [], None

    draw_corners = [marker_corners for _, marker_corners in detections]
    draw_ids = np.array([marker_id for marker_id, _ in detections], dtype=np.int32).reshape(-1, 1)
    return draw_corners, draw_ids


def create_video_writer(output_video_path, frame_shape, export_fps):
    output_height, output_width = frame_shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    return cv2.VideoWriter(
        output_video_path,
        fourcc,
        export_fps,
        (output_width, output_height),
    )


def main():
    args = parse_args()
    show_display = not args.no_display

    if not os.path.exists(args.video):
        raise FileNotFoundError(f"Video file not found: {args.video}")

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video file: {args.video}")

    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
    detector_params = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(dictionary, detector_params)

    source_fps = cap.get(cv2.CAP_PROP_FPS)
    if not source_fps or source_fps <= 0:
        source_fps = 60.0

    frame_step = max(1, args.frame_step)
    export_fps = source_fps / frame_step
    if export_fps <= 0:
        export_fps = 30.0

    base_name = os.path.splitext(os.path.basename(args.video))[0]
    output_dir = os.path.dirname(args.video)
    output_video_path = os.path.join(output_dir, f"{base_name}_tracked_tags_annotated.mp4")

    video_writer = None
    processed_frames = 0
    point_sums = {}
    point_counts = {}
    fixed_runway_points = None

    print("เริ่มตรวจจับตำแหน่ง AR tag ID 0-11...")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("จบไฟล์วิดีโอ")
            break

        frame_index = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        if frame_index % frame_step != 0:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray)
        tracked_detections = build_tracked_detections(corners, ids)
        tracked_detections, synthetic_ids, duplicate_fix_applied = remap_duplicate_runway_ids(
            tracked_detections,
            args.duplicate_id6_mode,
        )
        tracked_positions = build_marker_positions(tracked_detections)
        runway_positions = runway_positions_from_all(tracked_positions)

        if fixed_runway_points is None:
            if runway_positions:
                processed_frames += 1
                update_average_accumulators(runway_positions, point_sums, point_counts)
            averaged_runway_points = build_average_points(point_sums, point_counts)
            draw_runway_points = estimate_points_for_drawing(averaged_runway_points)
            if processed_frames >= CALIBRATION_FRAME_COUNT:
                fixed_runway_points = dict(draw_runway_points)
                print(f"ตรึงกริดรันเวย์หลังเฉลี่ยครบ {CALIBRATION_FRAME_COUNT} เฟรมที่ประมวลผล")
        else:
            draw_runway_points = fixed_runway_points

        draw_corners, draw_ids = build_draw_marker_inputs(tracked_detections)
        if draw_ids is not None:
            aruco.drawDetectedMarkers(frame, draw_corners, draw_ids)

        draw_runway_grid(frame, draw_runway_points, fixed_runway_points is not None)
        draw_marker_positions(frame, tracked_positions, synthetic_ids)

        for line_index, text in enumerate(
            format_status_lines(
                tracked_positions,
                processed_frames,
                fixed_runway_points,
                duplicate_fix_applied,
                args.duplicate_id6_mode,
            )
        ):
            cv2.putText(
                frame,
                text,
                (20, 35 + (line_index * 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                TEXT_COLOR,
                2,
            )

        if frame.shape[0] > frame.shape[1]:
            frame_resized = resize_to_fit(frame, 540, 960)
        else:
            frame_resized = resize_to_fit(frame, args.max_width, args.max_height)

        if video_writer is None:
            video_writer = create_video_writer(output_video_path, frame_resized.shape, export_fps)

        if video_writer.isOpened():
            video_writer.write(frame_resized)

        if show_display:
            cv2.imshow("AR Tag Position Check", frame_resized)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    if video_writer is not None:
        video_writer.release()
    cv2.destroyAllWindows()

    print("\n--- สรุปผล ---")
    print(f"บันทึกวิดีโอผลลัพธ์ไปที่: {output_video_path}")


if __name__ == "__main__":
    main()