from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    defaults = dict(platform='patient_transport', condition='', method='proposed',
                    stage='check', allow_motion='false', headless='true',
                    sdk_python='', output='')
    executable = Path(get_package_prefix('cics_experiments')) / 'lib/cics_experiments/experiment'
    process = ExecuteProcess(
        cmd=[str(executable), *[part for name in defaults
                               for part in ('--' + name.replace('_', '-'), LaunchConfiguration(name))]],
        name='cics_experiment', output='screen')

    def finished(event, context):
        if event.returncode:
            raise RuntimeError('CICS experiment exited with code ' + str(event.returncode))
        return [EmitEvent(event=Shutdown(reason='CICS selected workflow completed'))]

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        RegisterEventHandler(OnProcessExit(target_action=process, on_exit=finished)),
        process,
    ])
