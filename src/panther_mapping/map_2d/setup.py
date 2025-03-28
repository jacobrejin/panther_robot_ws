import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'map_2d'


# all the data files, individual ones
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
    ]



# custom function to copy folders to the install directory
def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=package_files(data_files, ['world/', 'config/']),

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rejin',
    maintainer_email='rejin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_world_baselink_pose_pub = panther_transforms.uwb_world_baselink_pose_pub:main',
            'uwb_world_to_odom_tf = panther_transforms.uwb_world_to_odom_tf:main',
            'uwb_map_to_odom_tf = panther_transforms.uwb_map_to_odom_tf:main',
        ],
    },
)
