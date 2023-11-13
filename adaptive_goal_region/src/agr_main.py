import numpy as np

from adaptive_goal_region.src.agr_clustering import (
    grasping_poses_to_position_and_orientation,
    cluster_by_position,
    generate_indices_array,
)
from adaptive_goal_region.src.agr_helper import (
    closest_position_index,
    quaternions_to_euler,
)
from adaptive_goal_region.src.agr_interpolation import generate_interpolation_edges


def agr(pred_grasps_cam: np.ndarray) -> np.ndarray:

    #
    # Adaptive Goal Region
    #

    positions, orientations, matrices = grasping_poses_to_position_and_orientation(pred_grasps_cam)

    # Cluster by Position
    labels = cluster_by_position(positions)
    n_clusters = len(set(labels))
    print(f"Estimated number of clusters: {n_clusters}")

    # Cluster by Orientation and Generate Unique Labels
    indices, global_labels = generate_indices_array(labels, n_clusters, positions, orientations)

    # Draw Spline
    adaptive_goal_region_data = []
    for parent_cluster in global_labels:
        for sub_cluster in global_labels[parent_cluster]:
            label = global_labels[parent_cluster][sub_cluster]
            sub_cluster_positions = np.array([
                positions[matrices_index]
                for matrices_index, global_label in indices
                if global_label == label
            ])
            if np.any(sub_cluster_positions):
                line_start, line_end = generate_interpolation_edges(sub_cluster_positions)
                ind_start = closest_position_index(positions, line_start)
                ind_end = closest_position_index(positions, line_end)

                line_start_pos = positions[ind_start]
                line_start_ori = orientations[ind_start]
                line_end_pos = positions[ind_end]
                line_end_ori = orientations[ind_end]

                adaptive_goal_region_data.append(
                    np.concatenate([
                        line_start_pos,
                        quaternions_to_euler(line_start_ori),
                        line_end_pos,
                        quaternions_to_euler(line_end_ori),
                    ])
                )

                #
                # if draw:
                #     interpolated = generate_intermediate_poses(
                #         line_start_pos,
                #         line_start_ori,
                #         line_end_pos,
                #         line_end_ori,
                #         50,
                #     )
                #     for interpolated_matrix in interpolated:
                #         draw_grasps(
                #             [interpolated_matrix],
                #             np.eye(4),
                #             color=COLORS[label],
                #             gripper_openings=gripper_openings_k,
                #         )
    return np.array(adaptive_goal_region_data)
