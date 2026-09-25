"""Run against patched controller_manager.cpp; compiles its actual filtering loop."""
from pathlib import Path
import subprocess
import sys
import tempfile

source = Path(sys.argv[1]).read_text()
start = source.index('  for (const std::string & arg : cm_node_options_.arguments())')
end = source.index('  // Add deprecation notice', start)
loop = source[start:end].replace('cm_node_options_.arguments()', 'args')
program = r'''
#include <cassert>
#include <string>
#include <vector>
const std::string RCL_REMAP_FLAG="--remap", RCL_SHORT_REMAP_FLAG="-r";
const std::string RCL_PARAM_FLAG="--param", RCL_SHORT_PARAM_FLAG="-p";
int main() {
  std::vector<std::string> args{
    "--ros-args", "--params-file", "/tmp/launch_params__nsc3mcr",
    "--params-file", "/tmp/robot_description.yaml",
    "--params-file", "/tmp/launch_params__node42",
    "-r", "__ns:=/robot", "-r", "__node:=manager",
    "-p", "robot_description:=<robot/>"};
  std::vector<std::string> node_options_arguments;
''' + loop + r'''
  const std::vector<std::string> expected{
    "--ros-args", "--params-file", "/tmp/launch_params__nsc3mcr",
    "--params-file", "/tmp/robot_description.yaml",
    "--params-file", "/tmp/launch_params__node42"};
  assert(node_options_arguments == expected);
}
'''
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)
    (path / 'test.cpp').write_text(program)
    subprocess.run(['g++', '-std=c++17', str(path / 'test.cpp'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
print('controller argument filter: passed')
