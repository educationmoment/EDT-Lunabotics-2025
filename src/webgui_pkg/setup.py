from setuptools import find_packages, setup

package_name = 'webgui_pkg'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=['webgui_pkg'],
    package_data={'webgui_pkg': ['templates/*', 'static/**/*', 'static/**/**/*']},

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'Flask'],
    zip_safe=True,
    maintainer='edt',
    maintainer_email='catherineschuch0@gmail.com',
    description='Web GUI Server Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "webgui_server = webgui_pkg.app:app.run",
        ],
    },
)
