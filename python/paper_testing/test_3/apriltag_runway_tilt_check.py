import argparse
import csv
import os
import sys

import cv2
import cv2.aruco as aruco
import numpy as np

# Keep the Thai console output readable when stdout is redirected to a file:
# Windows falls back to cp1252 for pipes, which cannot encode Thai.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


DEFAULT_VIDEO_PATH = r"D:\THESIS\walk_test\walk.MOV"
ARUCO_DICT = aruco.DICT_6X6_250
TRACKED_TAG_IDS = set(range(12))
ROBOT_TAG_IDS = set(range(4))
RUNWAY_LEFT_IDS = [4, 6, 8, 10]
RUNWAY_RIGHT_IDS = [5, 7, 9, 11]
RUNWAY_ROW_PAIRS = [(4, 5), (6, 7), (8, 9), (10, 11)]
RUNWAY_IDS = set(RUNWAY_LEFT_IDS + RUNWAY_RIGHT_IDS)
RUNWAY_ROW_INDEX = {4: 0, 5: 0, 6: 1, 7: 1, 8: 2, 9: 2, 10: 3, 11: 3}
RUNWAY_WORLD_POINTS_CM = {
    4: (0.0, 0.0),
    5: (80.0, 0.0),
    6: (0.0, 40.0),
    7: (80.0, 40.0),
    8: (0.0, 80.0),
    9: (80.0, 80.0),
    10: (0.0, 120.0),
    11: (80.0, 120.0),
}
ROBOT_TAG_LOCAL_POINTS_CM = {
    0: (-10.0, -21.5),
    1: (10.0, -21.5),
    2: (-10.0, 21.5),
    3: (10.0, 21.5),
}
CALIBRATION_FRAME_COUNT = 100
# Known distances between the robot back tags, used to re-measure the runway
# mapping from inside the data itself (see measure_robot_tag_scale).
ROBOT_TAG_PAIRS_CM = (((0, 1), 20.0), ((2, 3), 20.0), ((0, 2), 43.0), ((1, 3), 43.0))
# Area actually covered by the runway markers; poses outside it are extrapolated.
RUNWAY_GRID_X_RANGE_CM = (0.0, 80.0)
RUNWAY_GRID_Y_RANGE_CM = (0.0, 120.0)
DUPLICATE_ID6_MODE_CHOICES = ("right-as-7", "auto", "off")
POSE_SMOOTHING_ALPHA = 0.25
POSE_ARROW_LENGTH_CM = 18.0
BIRDSEYE_SCALE_PX_PER_CM = 6.0
BIRDSEYE_MARGIN_PX = 40
MARKER_COLOR = (0, 255, 255)
TEXT_COLOR = (255, 255, 255)
CENTER_COLOR = (0, 200, 0)
GRID_COLOR = (255, 80, 80)
POSE_COLOR = (0, 165, 255)
BIRDSEYE_BG_COLOR = (245, 245, 245)
BIRDSEYE_GRID_COLOR = (180, 180, 180)
BIRDSEYE_TRAJECTORY_COLOR = (50, 50, 255)


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
    parser.add_argument(
        "--no-video",
        action="store_true",
        help="Skip writing the annotated video (much faster when only the pose CSV is needed)",
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="Where to write the pose track (default: <video>_pose.csv next to the video)",
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


def robot_detections_from_all(detections):
    return [(marker_id, marker_corners) for marker_id, marker_corners in detections if marker_id in ROBOT_TAG_IDS]


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


def build_runway_homography(runway_points):
    image_points = []
    world_points = []
    for marker_id, world_point in RUNWAY_WORLD_POINTS_CM.items():
        if marker_id not in runway_points:
            continue

        image_points.append(runway_points[marker_id])
        world_points.append(world_point)

    if len(image_points) < 4:
        return None

    homography, _ = cv2.findHomography(
        np.array(image_points, dtype=np.float32),
        np.array(world_points, dtype=np.float32),
        method=0,
    )
    return homography


def transform_point(point, homography):
    transformed = cv2.perspectiveTransform(
        np.array([[[point[0], point[1]]]], dtype=np.float32),
        homography,
    )
    return (float(transformed[0, 0, 0]), float(transformed[0, 0, 1]))


def marker_forward_world_vector(marker_corners, homography):
    points = marker_corners[0]
    center = marker_center(marker_corners)
    top_midpoint = ((points[0][0] + points[1][0]) * 0.5, (points[0][1] + points[1][1]) * 0.5)
    world_center = transform_point(center, homography)
    world_top = transform_point(top_midpoint, homography)
    forward_vector = np.array([world_top[0] - world_center[0], world_top[1] - world_center[1]], dtype=np.float64)
    norm = float(np.linalg.norm(forward_vector))
    if norm <= 1e-6:
        return None

    return forward_vector / norm


def rotation_matrix_from_forward(forward_vector):
    forward = np.array(forward_vector, dtype=np.float64)
    right = np.array([forward[1], -forward[0]], dtype=np.float64)
    return np.column_stack((right, forward))


def heading_from_rotation(rotation_matrix):
    forward = rotation_matrix @ np.array([0.0, 1.0], dtype=np.float64)
    return float(np.degrees(np.arctan2(forward[1], forward[0])))


def estimate_pose_from_correspondences(robot_world_points):
    if len(robot_world_points) < 2:
        return None

    visible_ids = sorted(robot_world_points.keys())
    local_points = np.array([ROBOT_TAG_LOCAL_POINTS_CM[marker_id] for marker_id in visible_ids], dtype=np.float64)
    world_points = np.array([robot_world_points[marker_id] for marker_id in visible_ids], dtype=np.float64)

    local_centroid = np.mean(local_points, axis=0)
    world_centroid = np.mean(world_points, axis=0)
    local_centered = local_points - local_centroid
    world_centered = world_points - world_centroid

    covariance = local_centered.T @ world_centered
    left_u, _, right_vt = np.linalg.svd(covariance)
    rotation = right_vt.T @ left_u.T
    if np.linalg.det(rotation) < 0:
        right_vt[-1, :] *= -1.0
        rotation = right_vt.T @ left_u.T

    translation = world_centroid - (rotation @ local_centroid)
    return {
        "center_world": (float(translation[0]), float(translation[1])),
        "heading_deg": heading_from_rotation(rotation),
        "visible_ids": visible_ids,
        "source": f"{len(visible_ids)}-tag rigid fit",
        "is_held": False,
    }


def estimate_pose_from_single_tag(marker_id, marker_corners, homography):
    if marker_id not in ROBOT_TAG_LOCAL_POINTS_CM:
        return None

    forward_vector = marker_forward_world_vector(marker_corners, homography)
    if forward_vector is None:
        return None

    world_center = np.array(transform_point(marker_center(marker_corners), homography), dtype=np.float64)
    rotation = rotation_matrix_from_forward(forward_vector)
    local_tag = np.array(ROBOT_TAG_LOCAL_POINTS_CM[marker_id], dtype=np.float64)
    robot_center = world_center - (rotation @ local_tag)
    return {
        "center_world": (float(robot_center[0]), float(robot_center[1])),
        "heading_deg": heading_from_rotation(rotation),
        "visible_ids": [marker_id],
        "source": "1-tag orientation fallback",
        "is_held": False,
    }


def estimate_robot_pose(robot_detections, homography):
    if homography is None or not robot_detections:
        return None

    robot_world_points = {}
    for marker_id, marker_corners in robot_detections:
        robot_world_points[marker_id] = transform_point(marker_center(marker_corners), homography)

    rigid_pose = estimate_pose_from_correspondences(robot_world_points)
    if rigid_pose is not None:
        return rigid_pose

    marker_id, marker_corners = robot_detections[0]
    return estimate_pose_from_single_tag(marker_id, marker_corners, homography)


def normalize_angle_deg(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def blend_angles_deg(previous_angle_deg, current_angle_deg, alpha):
    angle_delta = normalize_angle_deg(current_angle_deg - previous_angle_deg)
    return normalize_angle_deg(previous_angle_deg + (alpha * angle_delta))


def smooth_pose(previous_pose, current_pose, alpha):
    if previous_pose is None:
        return dict(current_pose)

    previous_center = previous_pose["center_world"]
    current_center = current_pose["center_world"]
    smoothed_center = (
        ((1.0 - alpha) * previous_center[0]) + (alpha * current_center[0]),
        ((1.0 - alpha) * previous_center[1]) + (alpha * current_center[1]),
    )
    smoothed_heading = blend_angles_deg(previous_pose["heading_deg"], current_pose["heading_deg"], alpha)
    smoothed_pose = dict(current_pose)
    smoothed_pose["center_world"] = smoothed_center
    smoothed_pose["heading_deg"] = smoothed_heading
    return smoothed_pose


def world_point_to_image(point, inverse_homography):
    return transform_point(point, inverse_homography)


def runway_world_bounds_cm():
    world_points = list(RUNWAY_WORLD_POINTS_CM.values())
    x_values = [point[0] for point in world_points]
    y_values = [point[1] for point in world_points]
    return min(x_values), max(x_values), min(y_values), max(y_values)


def build_world_to_birdeye_homography():
    min_x, max_x, min_y, max_y = runway_world_bounds_cm()
    width_px = int(round(((max_x - min_x) * BIRDSEYE_SCALE_PX_PER_CM) + (2 * BIRDSEYE_MARGIN_PX)))
    height_px = int(round(((max_y - min_y) * BIRDSEYE_SCALE_PX_PER_CM) + (2 * BIRDSEYE_MARGIN_PX)))
    homography = np.array(
        [
            [BIRDSEYE_SCALE_PX_PER_CM, 0.0, BIRDSEYE_MARGIN_PX - (min_x * BIRDSEYE_SCALE_PX_PER_CM)],
            [0.0, -BIRDSEYE_SCALE_PX_PER_CM, BIRDSEYE_MARGIN_PX + (max_y * BIRDSEYE_SCALE_PX_PER_CM)],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    return homography, (width_px, height_px)


def world_point_to_birdeye(point, world_to_birdeye_homography):
    return transform_point(point, world_to_birdeye_homography)


def draw_birdeye_grid(frame, world_to_birdeye_homography):
    for ordered_ids in (RUNWAY_LEFT_IDS, RUNWAY_RIGHT_IDS):
        for index in range(len(ordered_ids) - 1):
            start_world = RUNWAY_WORLD_POINTS_CM[ordered_ids[index]]
            end_world = RUNWAY_WORLD_POINTS_CM[ordered_ids[index + 1]]
            start = tuple(int(round(value)) for value in world_point_to_birdeye(start_world, world_to_birdeye_homography))
            end = tuple(int(round(value)) for value in world_point_to_birdeye(end_world, world_to_birdeye_homography))
            cv2.line(frame, start, end, BIRDSEYE_GRID_COLOR, 2)

    for left_id, right_id in RUNWAY_ROW_PAIRS:
        start_world = RUNWAY_WORLD_POINTS_CM[left_id]
        end_world = RUNWAY_WORLD_POINTS_CM[right_id]
        start = tuple(int(round(value)) for value in world_point_to_birdeye(start_world, world_to_birdeye_homography))
        end = tuple(int(round(value)) for value in world_point_to_birdeye(end_world, world_to_birdeye_homography))
        cv2.line(frame, start, end, BIRDSEYE_GRID_COLOR, 2)

    for marker_id, world_point in RUNWAY_WORLD_POINTS_CM.items():
        center = tuple(int(round(value)) for value in world_point_to_birdeye(world_point, world_to_birdeye_homography))
        cv2.circle(frame, center, 5, GRID_COLOR, -1)
        cv2.putText(
            frame,
            f"ID:{marker_id}",
            (center[0] + 8, center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            GRID_COLOR,
            1,
        )


def draw_birdeye_robot_pose(frame, pose, trajectory_world_points, world_to_birdeye_homography):
    if trajectory_world_points:
        trajectory_pixels = [
            tuple(int(round(value)) for value in world_point_to_birdeye(point, world_to_birdeye_homography))
            for point in trajectory_world_points
        ]
        if len(trajectory_pixels) >= 2:
            cv2.polylines(frame, [np.array(trajectory_pixels, dtype=np.int32)], False, BIRDSEYE_TRAJECTORY_COLOR, 2)

    if pose is None:
        return

    center_world = pose["center_world"]
    heading_rad = np.radians(pose["heading_deg"])
    forward_world = (
        center_world[0] + (POSE_ARROW_LENGTH_CM * np.cos(heading_rad)),
        center_world[1] + (POSE_ARROW_LENGTH_CM * np.sin(heading_rad)),
    )
    start = tuple(int(round(value)) for value in world_point_to_birdeye(center_world, world_to_birdeye_homography))
    end = tuple(int(round(value)) for value in world_point_to_birdeye(forward_world, world_to_birdeye_homography))
    cv2.circle(frame, start, 6, POSE_COLOR, -1)
    cv2.arrowedLine(frame, start, end, POSE_COLOR, 3, tipLength=0.25)


def build_birdeye_view(frame, image_to_world_homography, pose, trajectory_world_points, world_to_birdeye_homography, birdseye_size):
    width_px, height_px = birdseye_size
    birdseye_frame = np.full((height_px, width_px, 3), BIRDSEYE_BG_COLOR, dtype=np.uint8)
    if image_to_world_homography is None:
        cv2.putText(
            birdseye_frame,
            "Bird's-eye unavailable until runway calibration locks",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            TEXT_COLOR,
            2,
        )
        return birdseye_frame

    image_to_birdeye_homography = world_to_birdeye_homography @ image_to_world_homography
    warped_frame = cv2.warpPerspective(frame, image_to_birdeye_homography, (width_px, height_px))
    birdseye_frame = cv2.addWeighted(warped_frame, 0.82, birdseye_frame, 0.18, 0.0)
    draw_birdeye_grid(birdseye_frame, world_to_birdeye_homography)
    draw_birdeye_robot_pose(birdseye_frame, pose, trajectory_world_points, world_to_birdeye_homography)
    cv2.putText(
        birdseye_frame,
        "Bird's-eye runway plane",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        TEXT_COLOR,
        2,
    )
    return birdseye_frame


def resize_birdeye_to_match_main(main_frame, birdseye_frame):
    main_height, main_width = main_frame.shape[:2]
    birdseye_height, birdseye_width = birdseye_frame.shape[:2]
    if birdseye_height <= 0 or birdseye_width <= 0:
        return birdseye_frame

    scale = min(main_height / birdseye_height, main_width / birdseye_width)
    target_width = max(1, int(round(birdseye_width * scale)))
    target_height = max(1, int(round(birdseye_height * scale)))
    return cv2.resize(birdseye_frame, (target_width, target_height))


def compose_views(main_frame, birdseye_frame):
    target_height = max(main_frame.shape[0], birdseye_frame.shape[0])

    def pad_to_height(frame, height):
        if frame.shape[0] == height:
            return frame
        pad_bottom = height - frame.shape[0]
        return cv2.copyMakeBorder(frame, 0, pad_bottom, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))

    left = pad_to_height(main_frame, target_height)
    right = pad_to_height(birdseye_frame, target_height)
    return np.hstack((left, right))


def draw_robot_pose(frame, pose, inverse_homography):
    if pose is None or inverse_homography is None:
        return

    center_world = pose["center_world"]
    heading_rad = np.radians(pose["heading_deg"])
    forward_world = (
        center_world[0] + (POSE_ARROW_LENGTH_CM * np.cos(heading_rad)),
        center_world[1] + (POSE_ARROW_LENGTH_CM * np.sin(heading_rad)),
    )

    center_image = world_point_to_image(center_world, inverse_homography)
    forward_image = world_point_to_image(forward_world, inverse_homography)
    start_point = (int(round(center_image[0])), int(round(center_image[1])))
    end_point = (int(round(forward_image[0])), int(round(forward_image[1])))
    cv2.circle(frame, start_point, 6, POSE_COLOR, -1)
    cv2.arrowedLine(frame, start_point, end_point, POSE_COLOR, 3, tipLength=0.25)


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


def format_status_lines(positions, processed_frames, fixed_runway_points, duplicate_fix_applied, duplicate_id6_mode, robot_pose):
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

    if robot_pose is None:
        status_lines.append("Robot pose: unavailable")
    else:
        center_x, center_y = robot_pose["center_world"]
        status_lines.append(
            f"Robot pose: x={center_x:.1f} cm y={center_y:.1f} cm yaw={robot_pose['heading_deg']:.1f} deg"
        )
        status_lines.append(f"Robot source: {robot_pose['source']}")

    return status_lines


def build_draw_marker_inputs(detections):
    if not detections:
        return [], None

    draw_corners = [marker_corners for _, marker_corners in detections]
    draw_ids = np.array([marker_id for marker_id, _ in detections], dtype=np.int32).reshape(-1, 1)
    return draw_corners, draw_ids


def calibrate_runway_homography(video_path, frame_step, duplicate_id6_mode):
    """First pass: average the runway markers and lock the homography.

    Doing this before tracking means the pose track can start at frame 0.  When
    calibration was folded into the tracking loop it consumed the first
    CALIBRATION_FRAME_COUNT * frame_step frames, so several seconds of every
    walk went unrecorded.
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video file: {video_path}")

    aruco_detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(ARUCO_DICT),
        aruco.DetectorParameters(),
    )
    point_sums = {}
    point_counts = {}
    processed_frames = 0

    while cap.isOpened():
        # grab() advances without decoding, so skipped frames cost almost nothing
        if not cap.grab():
            break

        frame_index = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        if frame_index % frame_step != 0:
            continue

        ret, frame = cap.retrieve()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray)
        detections = build_tracked_detections(corners, ids)
        detections, _, _ = remap_duplicate_runway_ids(detections, duplicate_id6_mode)
        runway_positions = runway_positions_from_all(build_marker_positions(detections))
        if not runway_positions:
            continue

        processed_frames += 1
        update_average_accumulators(runway_positions, point_sums, point_counts)
        if processed_frames >= CALIBRATION_FRAME_COUNT:
            break

    cap.release()
    fixed_points = estimate_points_for_drawing(build_average_points(point_sums, point_counts))
    return dict(fixed_points), build_runway_homography(fixed_points), processed_frames


def measure_robot_tag_scale(positions, homography):
    """Re-measure the known robot tag rectangle through the runway homography.

    The four back tags form a rigid 20 x 43 cm rectangle, so projecting them and
    measuring the sides checks the mapping without any extra equipment.  A scale
    away from 1.0 means either the tags sit above the ground plane the runway
    markers define (parallax), or ROBOT_TAG_LOCAL_POINTS_CM does not match the
    physical layout - measure the real tag spacing to tell the two apart.
    """
    if homography is None:
        return None

    world = {
        marker_id: transform_point(positions[marker_id], homography)
        for marker_id in ROBOT_TAG_IDS
        if marker_id in positions
    }

    scales = {}
    for (first_id, second_id), true_cm in ROBOT_TAG_PAIRS_CM:
        if first_id not in world or second_id not in world:
            continue

        measured_cm = float(np.hypot(
            world[first_id][0] - world[second_id][0],
            world[first_id][1] - world[second_id][1],
        ))
        scales.setdefault(true_cm, []).append(measured_cm / true_cm)

    if not scales:
        return None

    return {true_cm: float(np.mean(values)) for true_cm, values in scales.items()}


def pose_is_inside_grid(center_world):
    x_min, x_max = RUNWAY_GRID_X_RANGE_CM
    y_min, y_max = RUNWAY_GRID_Y_RANGE_CM
    return bool(
        x_min <= center_world[0] <= x_max and y_min <= center_world[1] <= y_max
    )


def write_pose_csv(csv_path, pose_rows):
    if not pose_rows:
        print("ไม่มีข้อมูล pose ให้บันทึก")
        return None

    with open(csv_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(pose_rows[0].keys()))
        writer.writeheader()
        writer.writerows(pose_rows)
    return csv_path


def summarise_pose_track(pose_rows):
    measured_rows = [row for row in pose_rows if not row["is_held"]]
    if len(measured_rows) < 2:
        print("ข้อมูล pose ไม่พอสำหรับสรุปผล")
        return

    x_cm = np.array([row["x_cm"] for row in measured_rows], dtype=np.float64)
    y_cm = np.array([row["y_cm"] for row in measured_rows], dtype=np.float64)
    heading_deg = np.array([row["heading_deg"] for row in measured_rows], dtype=np.float64)
    time_s = np.array([row["t_s"] for row in measured_rows], dtype=np.float64)

    forward_sign = 1.0 if y_cm[-1] >= y_cm[0] else -1.0
    travel_cm = float(abs(y_cm[-1] - y_cm[0]))
    duration_s = float(time_s[-1] - time_s[0])
    lateral_cm = (x_cm - x_cm[0]) * forward_sign
    yaw_deg = ((heading_deg - heading_deg[0] + 180.0) % 360.0) - 180.0
    inside_count = sum(1 for row in measured_rows if row["inside_grid"])

    print(f"เฟรมที่วัด pose ได้จริง: {len(measured_rows)} "
          f"(อยู่ในกริดอ้างอิง {inside_count}, นอกกริด {len(measured_rows) - inside_count})")
    if duration_s > 0:
        print(f"ระยะตามแนวทางวิ่ง: {travel_cm:.1f} cm ใน {duration_s:.1f} s "
              f"(เฉลี่ย {travel_cm / duration_s:.1f} cm/s)")
    print(f"เบี่ยงเบนด้านข้างจากจุดเริ่ม: สุดท้าย {lateral_cm[-1]:+.1f} cm | "
          f"RMS {float(np.sqrt(np.mean(lateral_cm ** 2))):.1f} cm | "
          f"สูงสุด {float(np.max(np.abs(lateral_cm))):.1f} cm")
    print(f"เบี่ยงเบนเชิงมุมจากจุดเริ่ม: สุดท้าย {yaw_deg[-1]:+.1f} deg | "
          f"RMS {float(np.sqrt(np.mean(yaw_deg ** 2))):.1f} deg")
    if travel_cm > 1.0:
        print(f"อัตราการเบี่ยงต่อระยะทาง: {lateral_cm[-1] / (travel_cm / 100.0):+.1f} cm/m")


def summarise_tag_scale(scale_samples):
    if not scale_samples:
        return

    print("\nตรวจสอบสเกลจากกรอบป้ายบนหุ่น (ค่าที่ถูกต้องคือ 1.000):")
    worst_error = 0.0
    for true_cm in sorted(scale_samples):
        values = np.array(scale_samples[true_cm], dtype=np.float64)
        axis_label = "ด้านกว้าง" if true_cm < 30.0 else "ด้านยาว"
        print(f"  {axis_label} ({true_cm:.0f} cm): {values.mean():.3f} "
              f"(sd {values.std():.3f}, n={len(values)})")
        worst_error = max(worst_error, abs(float(values.mean()) - 1.0))

    if worst_error > 0.05:
        print(f"  คำเตือน: สเกลคลาดเคลื่อนถึง {100.0 * worst_error:.0f}% "
              "ระยะทุกค่าที่วัดได้จึงผิดไปตามสัดส่วนนี้")
        print("  สาเหตุที่เป็นไปได้: ป้ายบนหุ่นอยู่สูงจากระนาบพื้นที่ใช้สอบเทียบ (พารัลแลกซ์)")
        print("  หรือค่า ROBOT_TAG_LOCAL_POINTS_CM ไม่ตรงกับระยะป้ายจริง — วัดระยะป้ายจริงเพื่อแยกสองกรณี")


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

    # Opened only to read the frame rate; the tracking pass reopens it below
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
    output_csv_path = args.csv or os.path.join(output_dir, f"{base_name}_pose.csv")

    # Pass 1: lock the runway grid before tracking, so no walk frames are lost
    print("รอบที่ 1: สอบเทียบกริดรันเวย์...")
    cap.release()
    fixed_runway_points, runway_homography, calibration_frames = calibrate_runway_homography(
        args.video, frame_step, args.duplicate_id6_mode
    )
    if runway_homography is None:
        raise RuntimeError(
            "สร้าง homography ไม่สำเร็จ: ตรวจพบเครื่องหมายทางวิ่งไม่ครบ 4 จุด"
        )
    print(f"ตรึงกริดรันเวย์จาก {calibration_frames} เฟรมที่มีเครื่องหมายทางวิ่ง")

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video file: {args.video}")

    video_writer = None
    processed_frames = calibration_frames
    inverse_runway_homography = np.linalg.inv(runway_homography)
    smoothed_robot_pose = None
    trajectory_world_points = []
    pose_rows = []
    scale_samples = {}
    world_to_birdeye_homography, birdseye_size = build_world_to_birdeye_homography()

    print("รอบที่ 2: ตรวจจับตำแหน่ง AR tag ID 0-11 ตั้งแต่เฟรมแรก...")

    while cap.isOpened():
        if not cap.grab():
            print("จบไฟล์วิดีโอ")
            break

        frame_index = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        if frame_index % frame_step != 0:
            continue

        ret, frame = cap.retrieve()
        if not ret:
            print("จบไฟล์วิดีโอ")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray)
        tracked_detections = build_tracked_detections(corners, ids)
        tracked_detections, synthetic_ids, duplicate_fix_applied = remap_duplicate_runway_ids(
            tracked_detections,
            args.duplicate_id6_mode,
        )
        tracked_positions = build_marker_positions(tracked_detections)
        robot_detections = robot_detections_from_all(tracked_detections)

        draw_runway_points = fixed_runway_points

        current_robot_pose = estimate_robot_pose(robot_detections, runway_homography)
        if current_robot_pose is None and smoothed_robot_pose is not None:
            current_robot_pose = {
                "center_world": smoothed_robot_pose["center_world"],
                "heading_deg": smoothed_robot_pose["heading_deg"],
                "visible_ids": [],
                "source": "hold-last-pose",
                "is_held": True,
            }

        if current_robot_pose is not None:
            smoothed_robot_pose = smooth_pose(smoothed_robot_pose, current_robot_pose, POSE_SMOOTHING_ALPHA)
        else:
            smoothed_robot_pose = None

        draw_corners, draw_ids = build_draw_marker_inputs(tracked_detections)
        if draw_ids is not None:
            aruco.drawDetectedMarkers(frame, draw_corners, draw_ids)

        draw_runway_grid(frame, draw_runway_points, fixed_runway_points is not None)
        draw_marker_positions(frame, tracked_positions, synthetic_ids)
        draw_robot_pose(frame, smoothed_robot_pose, inverse_runway_homography)

        if smoothed_robot_pose is not None:
            trajectory_world_points.append(smoothed_robot_pose["center_world"])
            center_world = smoothed_robot_pose["center_world"]
            pose_rows.append({
                "frame": frame_index,
                "t_s": round(frame_index / source_fps, 4),
                "x_cm": round(center_world[0], 4),
                "y_cm": round(center_world[1], 4),
                "heading_deg": round(smoothed_robot_pose["heading_deg"], 4),
                "n_tags": len(current_robot_pose["visible_ids"]) if current_robot_pose else 0,
                "source": smoothed_robot_pose["source"],
                "is_held": int(bool(smoothed_robot_pose.get("is_held", False))),
                "inside_grid": int(pose_is_inside_grid(center_world)),
            })

        frame_scale = measure_robot_tag_scale(tracked_positions, runway_homography)
        if frame_scale:
            for true_cm, scale_value in frame_scale.items():
                scale_samples.setdefault(true_cm, []).append(scale_value)

        birdseye_frame = build_birdeye_view(
            frame,
            runway_homography,
            smoothed_robot_pose,
            trajectory_world_points,
            world_to_birdeye_homography,
            birdseye_size,
        )

        for line_index, text in enumerate(
            format_status_lines(
                tracked_positions,
                processed_frames,
                fixed_runway_points,
                duplicate_fix_applied,
                args.duplicate_id6_mode,
                smoothed_robot_pose,
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

        birdseye_frame = resize_birdeye_to_match_main(frame, birdseye_frame)
        combined_frame = compose_views(frame, birdseye_frame)

        if frame.shape[0] > frame.shape[1]:
            frame_resized = resize_to_fit(combined_frame, 1200, 960)
        else:
            frame_resized = resize_to_fit(combined_frame, args.max_width * 2, args.max_height)

        if not args.no_video:
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
    if args.no_video:
        print("ข้ามการบันทึกวิดีโอ (--no-video)")
    else:
        print(f"บันทึกวิดีโอผลลัพธ์ไปที่: {output_video_path}")

    if write_pose_csv(output_csv_path, pose_rows):
        print(f"บันทึกข้อมูล pose ไปที่: {output_csv_path}")
    summarise_pose_track(pose_rows)
    summarise_tag_scale(scale_samples)


if __name__ == "__main__":
    main()