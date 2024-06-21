from setuptools import find_packages, setup

package_name = 'gr2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='benedict',
    maintainer_email='amoakobenedict@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gr2_keyboard_teleop = gr2_control.gr2_keyboard_teleop:main'
        ],
    },
)
