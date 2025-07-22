from setuptools import setup

package_name = 'amr_multi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'amr_master',
        'yolo_fake_publisher',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='AMR patrol/firestop demo package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr_master = amr_master:main',
            'yolo_fake_publisher = yolo_fake_publisher:main',
        ],
    },
)
