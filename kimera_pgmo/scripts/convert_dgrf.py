from scipy.spatial.transform import Rotation as R
import numpy as np
import click


def compute_meas(pose, to):
    x, y, z, qx, qy, qz, qw = pose
    pose_matrix = np.eye(4)
    pose_matrix[:3, 3] = x, y, z
    pose_matrix[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()

    meas = pose_matrix[:3, :3].T @ (to - pose_matrix[:3, 3])
    return meas


def modify_dedge(original_line):
    """
    This function takes in a original dedge line string and converts it to and updated format
    original: key1 key2 x y z qx qy qz qw to-x to-y to-z covariances
    new: key1 key2 px py pz covariances
    """
    # Split the original line into parts
    parts = original_line.split()

    # Extract the relevant fields from the original format
    tag = parts[0]
    key1 = parts[1]
    key2 = parts[2]
    x, y, z = map(float, parts[3:6])  # Pose (x, y, z)
    qx, qy, qz, qw = map(float, parts[6:10])  # Quaternion (qx, qy, qz, qw)
    to_x, to_y, to_z = map(float, parts[10:13])  # Target position (to-x, to-y, to-z)
    covariances = " ".join(parts[13:])  # Covariances (everything after to-z)

    # Call compute_meas to get the new position (px, py, pz)
    pose = (x, y, z, qx, qy, qz, qw)  # Pose tuple
    to = (to_x, to_y, to_z)  # Target position tuple
    px, py, pz = compute_meas(pose, to)  # Get the new position

    # Format the new line with the new position and covariances
    new_line = f"{tag} {key1} {key2} {px} {py} {pz} {covariances}" + "\n"

    return new_line


def process_dgrf(input_dgrf, output_dgrf):
    """
    This function processes an input dgrf, modify lines that start with "DEDGE" (to update to new excepted format),
    and writes the result to an output file.
    Args:
        input_dgrf (str): Path to the input dgrf to be processed.
        output_dgrf (str): Path to the output dgrf where the result will be saved.
    """
    skipped_lines = 0

    # Open the input file for reading and the output file for writing
    with open(input_dgrf, "r") as infile, open(output_dgrf, "w") as outfile:
        # Iterate through each line in the input file
        for line in infile:
            if line.startswith("DEDGE"):
                line = modify_dedge(line)

            outfile.write(line)


@click.command()
@click.argument("input_dgrf", type=str)
@click.argument("output_dgrf", type=str)
def main(input_dgrf, output_dgrf):
    process_dgrf(input_dgrf, output_dgrf)


if __name__ == "__main__":
    main()
