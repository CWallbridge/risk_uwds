 #!/usr/bin/env python
 # -*- coding: utf-8 -*-
import os
from setuptools import setup
#from distutils.core import setup
#from distutils.command.install import INSTALL_SCHEMES

def readme():
    with open('README.md') as f:
        return f.read()

setup(name='risk_uwds',
      version='0.1.0',
      license='ISC',
      description='RISK Implementation using Underworlds.',
      long_description=readme(),
      url='https://github.com/CWallbridge/risk_uwds',
      author='Christopher D. Wallbridge',
      author_email='chris.wallbridge@googlemail.com',
      maintainer='Christopher D. Wallbridge',
      maintainer_email='chris.wallbridge@googlemail.com',
      #package_dir = {'src'},
      packages=['risk_uwds'],
      install_requires=["numpy","underworlds","klampt"],
      include_package_data=True
      #data_files=[('data', ['data/robot.xml']),
                    #('data', ['data/allLinks.txt']),
                    #('data', ['data/careObot.rob']),
                    #('data', ['data/human.rob']),
                    #('data', ['data/rArmLinks.txt']),
                    #('data', ['data/rArmLinksMinimal.txt']),
                    #('data', ['data/robotMinimal.txt']),
                    #('meshes', ['meshes/cafe_table_new.date']),
                    #('meshes', ['meshes/coke_can.dae']),
                    #('meshes', ['meshes/coke_can.png']),
                    #('meshes', ['meshes/colourlessCoke.dae'])
                    #]
)
