from setuptools import find_packages, setup

package_name = 'createmate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hello-robot',
    maintainer_email='dvarad2607@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_frame_listener=createmate.aruco_frame_listener:main',
            'keyboard_save_transform=createmate.keyboard_save_transform:main',
            'playback_recording=createmate.poses_to_motion_control:main',
            'draw_circle=createmate.draw_circle:main',
            'write_pose=createmate.write_curr_pose:main',
            'nav2markers=createmate.navigate_to_markers:main',
            'posemanipulate=createmate.move_to_preset_poses:main',
            'sleepyaction=createmate.sleepy_action_server:main',
            'poselistener=createmate.pose_listener:main',
            'navigatorsrv=createmate.navigator:main',
            'coordinator=createmate.coordinator:main',
            'draw_service=createmate.service_draw_shapes:main',
        ],
    },
)
