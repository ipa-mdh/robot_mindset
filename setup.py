from setuptools import find_packages, setup
from pathlib import Path

package_name = 'robot_mindset'

def package_files(directory: Path):
    """
    Recursively collect all files under the given directory.
    
    The returned list will contain file paths as strings relative
    to the specified directory. This helps preserve the directory
    structure when installing.
    """
    return [str(path.relative_to(directory)) for path in directory.rglob('*') if path.is_file()]

# Define the path to the config directory inside the package
config_dir = Path(package_name) / 'config'
# Get a list of all files in the config directory, preserving their relative paths
config_files = package_files(config_dir)

# Define the path to the image directory inside the package
image_dir = Path(package_name) / 'image'
# Get a list of all files in the config directory, preserving their relative paths
image_files = package_files(image_dir)

# Prepare data_files for installation
data_files = [
    # Register the package in the ament index
    (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    # Include the package.xml file
    (f'share/{package_name}', ['package.xml']),
    # Include all files from the config directory in the share directory
    (
        f'share/{package_name}/config',
        [str(config_dir / relative_file) for relative_file in config_files]
    ),
    # Include all image files in the share directory
    (
        f'share/{package_name}/image',
        [str(image_dir / relative_file) for relative_file in image_files]
    )
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = robot_mindset.main:main',
        ],
    },
)
