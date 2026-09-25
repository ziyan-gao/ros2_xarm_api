"""Task-owned flow and motion policy, imported anew when that task restarts."""
from importlib import import_module


def task_module(name):
    if name not in ('pack_new', 'unpack', 'pack_slot', 'repack'):
        raise ValueError('unknown task: '+str(name))
    return import_module(__name__+'.'+name)
