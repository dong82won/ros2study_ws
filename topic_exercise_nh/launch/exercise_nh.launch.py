from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_exercise_nh',  # 실행할 패키지 이름
            executable='exercise',         # 실행할 노드 이름
            #name='exercise_node',         # 노드 이름 (선택 사항)
            output='screen'),              # 출력 옵션
    ])
