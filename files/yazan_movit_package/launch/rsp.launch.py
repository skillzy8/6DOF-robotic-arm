from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("urdf_expoert_second_design", package_name="yazan_movit_package").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
