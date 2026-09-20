from types import SimpleNamespace as NS

from safe_servo_visualization.policy_loading_node import PolicyLoadingNode


def test_yaml_clearance_overrides_old_launch_parameter(monkeypatch):
    import safe_servo_visualization.policy_loading_node as module
    config = dict(checkpoint='dummy.pth', clearance=35, xy_resolution_mm=5)
    monkeypatch.setattr(module.OmegaConf, 'load', lambda _: config)
    monkeypatch.setattr(module.OmegaConf, 'to_container', lambda cfg, **kw: cfg)
    captured = {}
    monkeypatch.setattr(module, 'RealPlatformPolicyLoader', lambda **kw: captured.update(kw) or NS(**kw))
    node = object.__new__(PolicyLoadingNode)
    parameters = {'clearance_mm': 20, 'seed': 0}
    node.declare_parameter = lambda name, default: parameters.setdefault(name, default)
    node.get_parameter = lambda name: NS(value=parameters[name])
    def set_parameters(values):
        parameters.update({p.name: p.value for p in values})
        return [NS(successful=True) for _ in values]
    node.set_parameters = set_parameters
    node._build_loader((450, 550, 450))
    assert captured['clearance_mm'] == parameters['clearance_mm'] == 35
    assert captured['xy_resolution_mm'] == 5
    assert node.result_config['effective_clearance_mm'] == 35
    assert node.result_config['effective_xy_resolution_mm'] == 5
