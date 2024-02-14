from setuptools import setup

package_name = 'zed_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasper',
    maintainer_email='49565505+wluxie@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follow_node = zed_cv.lane_follow_node:main',
            'steering = zed_cv.steering:pid_control',
            'find_ROI = zed_cv.find_ROI:main',
            'utils = zed_cv.utils:find_lane',
            'utils = zed_cv.utils:wrap_img',
            'run_pc = zed_cv.run_pc:main',
        ],
    },
)
