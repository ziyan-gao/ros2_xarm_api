from pathlib import Path
import time

import rclpy
from omegaconf import OmegaConf
from std_srvs.srv import Trigger

from packing.real_platform_policy_loading import RealPlatformPolicyLoader

from .random_stable_loading_node import RandomStableLoadingNode


class PolicyLoadingNode(RandomStableLoadingNode):
    """Run learned-policy placements on measured items without MCTS/A*."""

    NODE_NAME = 'policy_loading'
    API_PREFIX = '/policy_loading'
    DEFAULT_VISUALIZATION_PORT = 8766
    DEFAULT_CLEARANCE_MM = 20
    DEFAULT_SEED = 0
    VISUALIZATION_DIRECTORY = '/tmp/policy_loading_visualization'
    LOADING_LABEL = 'policy loading'

    def __init__(self):
        super().__init__()
        self.create_service(
            Trigger, '/policy_loading/plan', self.plan_only_callback)

    def plan_only_callback(self, _request, response):
        """Estimate the item and publish a policy target without grasping it."""
        if self.state != 'IDLE' or self.loader.pending is not None:
            response.message = (
                f'policy loading is not idle; current state is {self.state}')
            return response
        if self.pallet_status != 'LOCKED':
            response.message = 'pallet pose is not LOCKED'
            return response
        pipeline_state = self.pipeline_status.get('state')
        if pipeline_state in self.ACTIVE_PIPELINE_STATES:
            response.message = f'PickAndPlace is already active in {pipeline_state}'
            return response
        if not self.object_info_start_client.service_is_ready():
            response.message = 'object-information estimation service is unavailable'
            return response

        self.state = 'LOCALIZING'
        self.fault = ''
        self.last_result = ''
        self.localization_started = time.monotonic()
        self.cycle_auto_start = False
        self.continuous_run_active = False
        self.abort_requested = False
        future = self.object_info_start_client.call_async(Trigger.Request())
        future.add_done_callback(self._localization_start_completed)
        response.success = True
        response.message = (
            'policy target planning started; PickAndPlace will remain stopped')
        self.publish_status()
        return response

    def _build_loader(self, container_size):
        self.declare_parameter(
            'policy_config_path',
            '/opt/neuromeka_bin_packing/configs/real_platform_policy.yaml')
        config_path = Path(
            str(self.get_parameter('policy_config_path').value)).expanduser()
        config = OmegaConf.to_container(
            OmegaConf.load(config_path), resolve=True)
        if not isinstance(config, dict):
            raise TypeError(f'policy config must be a mapping: {config_path}')
        if bool(config.get('enable_rearrangement', False)):
            raise ValueError(
                'real-platform policy stage requires enable_rearrangement=false')
        checkpoint = Path(str(config['checkpoint'])).expanduser()
        if not checkpoint.is_absolute():
            checkpoint = config_path.parent.parent / checkpoint
        self.declare_parameter(
            'checkpoint_path', str(checkpoint))
        self.declare_parameter(
            'policy_device', str(config.get('device', 'cpu')))
        self.declare_parameter(
            'k_placement', int(config.get('k_placement', 80)))
        self.declare_parameter(
            'height_tolerance', float(config.get('height_tolerance', 0.0)))
        self.declare_parameter(
            'remove_inscribed_ems',
            bool(config.get('remove_inscribed_ems', False)))
        self.declare_parameter(
            'ems_mode',
            str(config.get('ems_mode', 'clipped_ems_by_substantial_contact')))
        self.declare_parameter(
            'check_achievable_lps',
            bool(config.get('check_achievable_lps', False)))
        self.declare_parameter(
            'loading_mode', str(config.get('loading_mode', 'vertical_loading')))
        self.declare_parameter(
            'container_tracking', str(config.get('container_tracking', 'full')))
        return RealPlatformPolicyLoader(
            checkpoint_path=str(self.get_parameter('checkpoint_path').value),
            device=str(self.get_parameter('policy_device').value),
            container_size=container_size,
            clearance_mm=int(self.get_parameter('clearance_mm').value),
            height_tolerance=float(
                self.get_parameter('height_tolerance').value),
            seed=int(self.get_parameter('seed').value),
            k_placement=int(self.get_parameter('k_placement').value),
            remove_inscribed_ems=bool(
                self.get_parameter('remove_inscribed_ems').value),
            ems_mode=str(self.get_parameter('ems_mode').value),
            check_achievable_lps=bool(
                self.get_parameter('check_achievable_lps').value),
            loading_mode=str(self.get_parameter('loading_mode').value),
            container_tracking=str(
                self.get_parameter('container_tracking').value),
        )

    def _log_ready(self, container_size):
        self.get_logger().info(
            'policy loading ready (MCTS/A*=disabled): container=%s mm, '
            'clearance=%d mm, height_tolerance=%.1f mm, checkpoint=%s, '
            'device=%s' % (
                container_size,
                self.loader.clearance_mm,
                self.loader.height_tolerance,
                self.loader.checkpoint_path,
                getattr(self.loader.agent, 'device', self.loader.device),
            ))

    def _log_pending(self, pending):
        self.get_logger().info(
            'policy target %d: corner=(%d, %d, %d) mm, rotate_90=%s, '
            'action=%d, ems=%d, value=%.6f, virtual_dim=(%d, %d, %d) mm' % (
                pending.sequence_id,
                pending.box.FLB.x,
                pending.box.FLB.y,
                pending.box.FLB.z,
                pending.box.rot,
                pending.action_index,
                pending.ems_index,
                pending.predicted_value,
                pending.box.Virtual_Dim.dx,
                pending.box.Virtual_Dim.dy,
                pending.box.Virtual_Dim.dz,
            ))

    def _extend_status(self, payload, pending):
        payload.update({
            'selection_pipeline': 'learned_policy_only',
            'mcts_enabled': False,
            'astar_enabled': False,
            'checkpoint': str(self.loader.checkpoint_path),
            'device': str(getattr(self.loader.agent, 'device', self.loader.device)),
            'clearance_mm': int(self.loader.clearance_mm),
        })
        if pending is None:
            return
        payload.update({
            'target_corner_mm': [
                int(pending.box.FLB.x),
                int(pending.box.FLB.y),
                int(pending.box.FLB.z),
            ],
            'rotate_item_90_deg': bool(pending.box.rot),
            'raw_dimension_mm': list(map(int, pending.raw_dim.raw())),
            'virtual_dimension_mm': list(map(int, pending.box.Virtual_Dim.raw())),
            'policy_action_index': int(pending.action_index),
            'policy_ems_index': int(pending.ems_index),
            'policy_predicted_value': float(pending.predicted_value),
        })


def main(args=None):
    rclpy.init(args=args)
    node = PolicyLoadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
