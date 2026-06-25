import argparse
import os

import cv2
import cv2.aruco as aruco
import numpy as np


DEFAULT_VIDEO_PATH = r"D:\THESIS\walk_test\walk_0kg.MOV"
ARUCO_DICT = aruco.DICT_6X6_250
RUNWAY_LEFT_IDS = [4, 6, 8, 10]
RUNWAY_RIGHT_IDS = [5, 7, 9, 11]
RUNWAY_ROW_PAIRS = [(4, 5), (6, 7), (8, 9), (10, 11)]
RUNWAY_IDS = set(RUNWAY_LEFT_IDS + RUNWAY_RIGHT_IDS)
RUNWAY_ROW_INDEX = {4: 0, 5: 0, 6: 1, 7: 1, 8: 2, 9: 2, 10: 3, 11: 3}
POINT_SMOOTHING_ALPHA = 0.35


def parse_args():
    parser = argparse.ArgumentParser(
        description="Draw a robust runway grid from runway AprilTags, even when some IDs disappear."
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


def build_marker_lookup(corners, ids):
    if ids is None:
        return {}

    lookup = {}
    for index, marker_id in enumerate(ids.flatten()):
        lookup[int(marker_id)] = marker_center(corners[index])
    return lookup


def blend_point(previous_point, current_point, alpha):
    if previous_point is None:
        return current_point

    prev_x, prev_y = previous_point
    curr_x, curr_y = current_point
    blended_x = ((1.0 - alpha) * prev_x) + (alpha * curr_x)
    blended_y = ((1.0 - alpha) * prev_y) + (alpha * curr_y)
    return (blended_x, blended_y)


def fit_axis_from_rows(points_by_id):
    if len(points_by_id) < 2:
        return None, None

    row_values = np.array([RUNWAY_ROW_INDEX[marker_id] for marker_id in points_by_id], dtype=np.float64)
    x_values = np.array([points_by_id[marker_id][0] for marker_id in points_by_id], dtype=np.float64)
    y_values = np.array([points_by_id[marker_id][1] for marker_id in points_by_id], dtype=np.float64)

    x_model = np.polyfit(row_values, x_values, 1)
    y_model = np.polyfit(row_values, y_values, 1)
    return x_model, y_model


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


def update_seen_points(marker_lookup, seen_points):
    visible_points = {marker_id: marker_lookup[marker_id] for marker_id in RUNWAY_IDS if marker_id in marker_lookup}
    for marker_id, point in visible_points.items():
        previous_point = seen_points.get(marker_id)
        seen_points[marker_id] = blend_point(previous_point, point, POINT_SMOOTHING_ALPHA)
    return visible_points


def estimate_points_for_drawing(seen_points):
    # seen_points holds only real (remembered) observations and must not be polluted
    # by estimates, so we work on a separate dict for drawing.
    draw_points = dict(seen_points)

    for _ in range(2):
        estimate_missing_side_points(RUNWAY_LEFT_IDS, draw_points)
        estimate_missing_side_points(RUNWAY_RIGHT_IDS, draw_points)
        fill_missing_row_partners(draw_points)

    return draw_points


def draw_polyline_by_ids(frame, ordered_ids, points_by_id, color, thickness):
    available_ids = [marker_id for marker_id in ordered_ids if marker_id in points_by_id]
    for index in range(len(available_ids) - 1):
        start = tuple(int(round(value)) for value in points_by_id[available_ids[index]])
        end = tuple(int(round(value)) for value in points_by_id[available_ids[index + 1]])
        cv2.line(frame, start, end, color, thickness)


def point_category(marker_id, visible_points, seen_points):
    if marker_id in visible_points:
        return "visible"
    if marker_id in seen_points:
        return "remembered"
    return "estimated"


def draw_runway_grid(frame, draw_points, visible_points, seen_points):
    grid_color = (255, 80, 80)
    category_color = {
        "visible": (0, 255, 255),
        "remembered": (0, 255, 0),
        "estimated": (120, 120, 255),
    }

    draw_polyline_by_ids(frame, RUNWAY_LEFT_IDS, draw_points, grid_color, 2)
    draw_polyline_by_ids(frame, RUNWAY_RIGHT_IDS, draw_points, grid_color, 2)

    for left_id, right_id in RUNWAY_ROW_PAIRS:
        if left_id not in draw_points or right_id not in draw_points:
            continue

        start = tuple(int(round(value)) for value in draw_points[left_id])
        end = tuple(int(round(value)) for value in draw_points[right_id])
        cv2.line(frame, start, end, grid_color, 2)

    for marker_id, point in draw_points.items():
        category = point_category(marker_id, visible_points, seen_points)
        color = category_color[category]
        radius = 5 if category == "visible" else 4
        center = (int(round(point[0])), int(round(point[1])))
        cv2.circle(frame, center, radius, color, -1)
        label = f"ID:{marker_id}" if category == "visible" else f"ID:{marker_id}*"
        cv2.putText(
            frame,
            label,
            (center[0] + 8, center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )


def format_runway_status(visible_points, seen_points, draw_points):
    visible_ids = sorted(visible_points.keys())
    remembered_ids = sorted(marker_id for marker_id in seen_points if marker_id not in visible_points)
    estimated_ids = sorted(marker_id for marker_id in draw_points if marker_id not in seen_points)
    return [
        f"Runway visible: {visible_ids}",
        f"Runway remembered: {remembered_ids}",
        f"Runway estimated: {estimated_ids}",
        "Legend: ID* = remembered/estimated (not seen this frame)",
    ]


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
    output_video_path = os.path.join(output_dir, f"{base_name}_runway_grid_annotated.mp4")

    video_writer = None
    seen_points = {}

    print("เริ่มตรวจจับแท็กรันเวย์และตีเส้นกริด...")

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
        marker_lookup = build_marker_lookup(corners, ids)

        visible_points = update_seen_points(marker_lookup, seen_points)
        draw_points = estimate_points_for_drawing(seen_points)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        draw_runway_grid(frame, draw_points, visible_points, seen_points)

        for line_index, text in enumerate(format_runway_status(visible_points, seen_points, draw_points)):
            cv2.putText(
                frame,
                text,
                (20, 35 + (line_index * 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 255, 255),
                2,
            )

        if frame.shape[0] > frame.shape[1]:
            frame_resized = resize_to_fit(frame, 540, 960)
        else:
            frame_resized = resize_to_fit(frame, args.max_width, args.max_height)

        if video_writer is None:
            output_height, output_width = frame_resized.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            video_writer = cv2.VideoWriter(
                output_video_path,
                fourcc,
                export_fps,
                (output_width, output_height),
            )

        if video_writer.isOpened():
            video_writer.write(frame_resized)

        if show_display:
            cv2.imshow("Runway Grid Check", frame_resized)
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