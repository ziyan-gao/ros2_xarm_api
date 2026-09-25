"""Build-time opt-in patch, against a checked upstream launch anchor."""
from pathlib import Path

path = Path('/opt/xarm_ws/install/xarm_moveit_config/share/xarm_moveit_config/launch/_robot_moveit_common2.launch.py')
source = path.read_text()
anchor = '    moveit_config_package_name = \'xarm_moveit_config\''
insertion = '''
    if __import__('os').environ.get('CUMOTION_LIVE_ENABLED') == 'true':
        pipelines = moveit_config_dict.setdefault('planning_pipelines', ['ompl'])
        if 'isaac_ros_cumotion' not in pipelines:
            pipelines.append('isaac_ros_cumotion')
        default_pipeline = __import__('os').environ.get(
            'CUMOTION_DEFAULT_PLANNING_PIPELINE', 'ompl')
        if default_pipeline not in pipelines:
            raise ValueError('Unknown default planning pipeline: ' + default_pipeline)
        moveit_config_dict['default_planning_pipeline'] = default_pipeline
        moveit_config_dict['isaac_ros_cumotion'] = {
            'planning_plugins': ['isaac_ros_cumotion_moveit/CumotionPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision'],
            'response_adapters': [
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath']}
        moveit_config_dict['allow_trajectory_execution'] = (
            __import__('os').environ.get('CUMOTION_ALLOW_TRAJECTORY_EXECUTION', 'false') == 'true')
'''
if source.count(anchor) != 1 or 'CUMOTION_LIVE_ENABLED' in source:
    raise RuntimeError('Unexpected upstream launch; refusing blind patch')
path.write_text(source.replace(anchor, anchor+'\n'+insertion))
